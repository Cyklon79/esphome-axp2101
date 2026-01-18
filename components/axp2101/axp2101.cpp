#include "axp2101.h"
#include "esphome/core/log.h"

namespace esphome {
namespace axp2101 {

static const char *TAG = "axp2101";

// Rejestry, których używamy (z Twoich działających poprawek + typowy układ ADC AXP)
static const uint8_t REG_LDO_EN_0x90   = 0x90;  // enable bits (BLDO1 bit4 wg Twojego testu)
static const uint8_t REG_BLDO1_CFG_0x96 = 0x96; // BLDO1 voltage step (0..31)
static const uint8_t REG_ADC_EN_0x82   = 0x82;  // ADC enable (typowo w rodzinie AXP)

static const uint8_t REG_STATUS_0x01   = 0x01;  // status (tu logujemy surowo; charging bit może się różnić)
static const uint8_t REG_VBAT_0x78     = 0x78;  // VBAT ADC MSB (12-bit)
static const uint8_t REG_VBAT_0x79     = 0x79;  // VBAT ADC LSB (low nibble)

bool AXP2101Component::read_u8_(uint8_t reg, uint8_t &val) {
  return this->read_byte(reg, &val);
}

bool AXP2101Component::write_u8_(uint8_t reg, uint8_t val) {
  return this->write_byte(reg, val);
}

bool AXP2101Component::read_buf_(uint8_t reg, uint8_t *buf, size_t len) {
  return this->read_bytes(reg, buf, len);
}

float AXP2101Component::get_setup_priority() const {
  // DATA jest OK: chcemy być gotowi wcześnie, zanim display zacznie rysować
  return setup_priority::DATA;
}

void AXP2101Component::dump_config() {
  ESP_LOGCONFIG(TAG, "AXP2101:");
  LOG_I2C_DEVICE(this);
  LOG_SENSOR("  ", "Battery Voltage", this->batteryvoltage_sensor_);
  LOG_SENSOR("  ", "Battery Level", this->batterylevel_sensor_);
  LOG_BINARY_SENSOR("  ", "Battery Charging", this->batterycharging_bsensor_);
  ESP_LOGCONFIG(TAG, "  Brightness: %.1f%%", this->brightness_ * 100.0f);
}

void AXP2101Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up AXP2101...");

  // 1) Włącz ADC (żeby VBAT nie wracało jako 0)
  //    Nie znamy dokładnie maski dla każdej funkcji ADC w AXP2101,
  //    ale 0xFF jest bezpieczne w praktyce do diagnostyki.
  if (!write_u8_(REG_ADC_EN_0x82, 0xFF)) {
    ESP_LOGW(TAG, "Failed to enable ADCs (reg 0x82). Battery voltage may read as 0.");
  }

  // 2) Ustaw BLDO1 na 3.3V (step 28 = 0x1C) i włącz BLDO1 (0x90 bit4)
  set_bldo1_voltage_step_(28);
  ensure_bldo1_enabled_();

  // 3) Zastosuj jasność z konfiguracji od razu przy starcie
  curr_brightness_ = -1.0f;
  update_brightness_();
}

void AXP2101Component::ensure_bldo1_enabled_() {
  uint8_t reg90{};
  if (!read_u8_(REG_LDO_EN_0x90, reg90)) {
    ESP_LOGW(TAG, "Failed to read reg 0x90 (enable outputs)");
    return;
  }
  uint8_t new90 = reg90 | 0x10;  // BLDO1 enable (bit4) - zgodnie z Twoim działającym testem
  if (new90 != reg90) {
    if (!write_u8_(REG_LDO_EN_0x90, new90)) {
      ESP_LOGW(TAG, "Failed to write reg 0x90 (enable BLDO1)");
      return;
    }
  }
  ESP_LOGD(TAG, "BLDO1 enable: reg90 0x%02X -> 0x%02X", reg90, new90);
}

void AXP2101Component::set_bldo1_voltage_step_(uint8_t step) {
  step &= 0x1F;  // 0..31
  uint8_t reg96{};
  if (!read_u8_(REG_BLDO1_CFG_0x96, reg96)) {
    // jeżeli nie umiemy odczytać, spróbujmy chociaż ustawić “w ciemno”
    uint8_t val = step;
    write_u8_(REG_BLDO1_CFG_0x96, val);
    ESP_LOGD(TAG, "BLDO1 cfg write (no read) reg96=0x%02X", val);
    return;
  }

  // Dolne 5 bitów to step, górne zostawiamy
  uint8_t new96 = (reg96 & 0xE0) | step;
  if (!write_u8_(REG_BLDO1_CFG_0x96, new96)) {
    ESP_LOGW(TAG, "Failed to write reg 0x96 (BLDO1 voltage step)");
    return;
  }
  ESP_LOGD(TAG, "BLDO1 voltage step: reg96 0x%02X -> 0x%02X (step=%u)", reg96, new96, step);
}

void AXP2101Component::update_brightness_() {
  if (brightness_ == curr_brightness_)
    return;

  curr_brightness_ = brightness_;
  ESP_LOGD(TAG, "Brightness request: %.1f%%", brightness_ * 100.0f);

  if (brightness_ <= 0.0f) {
    // wyłącz BLDO1
    uint8_t reg90{};
    if (read_u8_(REG_LDO_EN_0x90, reg90)) {
      uint8_t new90 = reg90 & ~0x10;
      write_u8_(REG_LDO_EN_0x90, new90);
      ESP_LOGD(TAG, "BLDO1 disabled: reg90 0x%02X -> 0x%02X", reg90, new90);
    }
    return;
  }

  // Zapewnij, że BLDO1 jest włączone
  ensure_bldo1_enabled_();

  // Mapowanie “widocznego” zakresu stepów:
  // Twoje testy pokazały, że sensownie świeci okolica 3.0–3.5V.
  // 0.5V + step*0.1V => 3.0V => step=25, 3.5V => step=30
  const uint8_t min_step = 25;
  const uint8_t max_step = 30;

  float b = brightness_;
  if (b > 1.0f) b = 1.0f;

  uint8_t step = static_cast<uint8_t>(min_step + (max_step - min_step) * b + 0.5f);
  if (step < min_step) step = min_step;
  if (step > max_step) step = max_step;

  set_bldo1_voltage_step_(step);
}

bool AXP2101Component::read_batt_voltage_v_(float &volts) {
  uint8_t buf[2]{};
  if (!read_buf_(REG_VBAT_0x78, buf, 2)) {
    ESP_LOGW(TAG, "Failed to read VBAT regs 0x78..0x79");
    return false;
  }

  // 12-bit: [0x78]=MSB, [0x79] low nibble
  uint16_t raw = (static_cast<uint16_t>(buf[0]) << 4) | (buf[1] & 0x0F);

  // LSB w rodzinie AXP często bywa ~1.1mV/bit (AXP192). Dla AXP2101 bywa podobnie.
  // Dlatego:
  float mv = raw * 1.1f;

  volts = mv / 1000.0f;

  ESP_LOGD(TAG, "VBAT raw: 0x%02X 0x%02X -> raw=%u -> %.3f V", buf[0], buf[1], raw, volts);
  return true;
}

float AXP2101Component::estimate_battery_percent_(float v) {
  // Prosta krzywa Li-Ion (heurystyka). Lepsze niż liniowe 3.0..4.1
  // Możesz potem dostroić pod swój pakiet.
  struct Pt { float v; float p; };
  static const Pt curve[] = {
    {4.20f, 100.f},
    {4.10f,  90.f},
    {4.00f,  75.f},
    {3.90f,  60.f},
    {3.80f,  40.f},
    {3.70f,  20.f},
    {3.60f,  10.f},
    {3.50f,   5.f},
    {3.30f,   0.f},
  };

  if (v >= curve[0].v) return 100.f;
  if (v <= curve[sizeof(curve)/sizeof(curve[0]) - 1].v) return 0.f;

  for (size_t i = 0; i < (sizeof(curve)/sizeof(curve[0]) - 1); i++) {
    if (v <= curve[i].v && v >= curve[i+1].v) {
      float t = (v - curve[i+1].v) / (curve[i].v - curve[i+1].v);
      return curve[i+1].p + t * (curve[i].p - curve[i+1].p);
    }
  }
  return 0.f;
}

void AXP2101Component::update() {
  // 1) VBAT
  float vbat{};
  if (read_batt_voltage_v_(vbat)) {
    if (batteryvoltage_sensor_ != nullptr)
      batteryvoltage_sensor_->publish_state(vbat);

    if (batterylevel_sensor_ != nullptr) {
      float pct = estimate_battery_percent_(vbat);
      if (pct < 0.f) pct = 0.f;
      if (pct > 100.f) pct = 100.f;
      batterylevel_sensor_->publish_state(pct);
    }
  }

  // 2) Charging – na razie: log surowego statusu + próba bitu (do weryfikacji)
  if (batterycharging_bsensor_ != nullptr) {
    uint8_t st{};
    if (read_u8_(REG_STATUS_0x01, st)) {
      // UWAGA: bit charging może się różnić; ten był w Twoim starym kodzie jako 0x20 (ale tam był błąd OR zamiast AND)
      bool charging = (st & 0x20) != 0;
      ESP_LOGD(TAG, "STATUS reg0x01=0x%02X -> charging(bit0x20)=%s", st, charging ? "true" : "false");
      batterycharging_bsensor_->publish_state(charging);
    } else {
      batterycharging_bsensor_->publish_state(false);
    }
  }

  // 3) Brightness
  update_brightness_();
}

}  // namespace axp2101
}  // namespace esphome
