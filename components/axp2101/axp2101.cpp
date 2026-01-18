#include "axp2101.h"
#include "esp_sleep.h"
#include "esphome/core/log.h"
#include <Esp.h>

// Defaults for Core2
#ifndef CONFIG_PMU_SDA
#define CONFIG_PMU_SDA 21
#endif

#ifndef CONFIG_PMU_SCL
#define CONFIG_PMU_SCL 22
#endif

#ifndef CONFIG_PMU_IRQ
#define CONFIG_PMU_IRQ 35
#endif

static bool pmu_flag = false;
static XPowersPMU PMU;

static const uint8_t i2c_sda = CONFIG_PMU_SDA;
static const uint8_t i2c_scl = CONFIG_PMU_SCL;
static const uint8_t pmu_irq_pin = CONFIG_PMU_IRQ;

static void setFlag() { pmu_flag = true; }

namespace esphome {
namespace axp2101 {

static const char *TAG = "axp2101.sensor";

void AXP2101Component::setup() {
  // NOTE: In many sketches XPowersLib expects you to call PMU.begin(...).
  // This code (as in the upstream snippets you pasted) assumes PMU is already usable.
  // If you hit issues later, we will explicitly init PMU with the correct I2C.
  ESP_LOGCONFIG(TAG, "getID:0x%x", PMU.getChipID());

  // VBUS safety limits
  PMU.setVbusVoltageLimit(XPOWERS_AXP2101_VBUS_VOL_LIM_4V36);
  PMU.setVbusCurrentLimit(XPOWERS_AXP2101_VBUS_CUR_LIM_1500MA);

  // VSYS shutdown voltage
  uint16_t vol = PMU.getSysPowerDownVoltage();
  ESP_LOGCONFIG(TAG, "-> getSysPowerDownVoltage:%u", vol);
  PMU.setSysPowerDownVoltage(2600);
  vol = PMU.getSysPowerDownVoltage();
  ESP_LOGCONFIG(TAG, "-> getSysPowerDownVoltage:%u", vol);

  // Rails configuration (keep as-is from v1 baseline)
  PMU.setDC1Voltage(3300);
  PMU.setDC2Voltage(1000);
  PMU.setDC3Voltage(3300);
  PMU.setDC4Voltage(1000);
  PMU.setDC5Voltage(3300);

  PMU.setALDO1Voltage(3300);
  PMU.setALDO2Voltage(3300);
  // PMU.setALDO3Voltage(3300); // speaker
  PMU.setALDO4Voltage(3300);

  // Backlight + extra rails
  PMU.setBLDO1Voltage(3300);  // Core2 v1.1 backlight rail target
  PMU.setBLDO2Voltage(3300);

  PMU.setCPUSLDOVoltage(1000);

  // Enable rails (keep conservative set; adjust later if you want)
  // PMU.enableDC1();
  PMU.enableDC2();
  PMU.enableDC3();
  PMU.enableDC4();
  PMU.enableDC5();
  PMU.enableALDO1();
  PMU.enableALDO2();
  // PMU.enableALDO3(); // speaker
  PMU.enableALDO4();

  // IMPORTANT for Core2 v1.1: enable BLDO1 (backlight)
  PMU.enableBLDO1();
  PMU.enableBLDO2();
  PMU.enableCPUSLDO();

  // Log rails state
  ESP_LOGCONFIG(TAG, "DC1  : %s Voltage:%u mV", PMU.isEnableDC1() ? "+" : "-", PMU.getDC1Voltage());
  ESP_LOGCONFIG(TAG, "DC2  : %s Voltage:%u mV", PMU.isEnableDC2() ? "+" : "-", PMU.getDC2Voltage());
  ESP_LOGCONFIG(TAG, "DC3  : %s Voltage:%u mV", PMU.isEnableDC3() ? "+" : "-", PMU.getDC3Voltage());
  ESP_LOGCONFIG(TAG, "DC4  : %s Voltage:%u mV", PMU.isEnableDC4() ? "+" : "-", PMU.getDC4Voltage());
  ESP_LOGCONFIG(TAG, "DC5  : %s Voltage:%u mV", PMU.isEnableDC5() ? "+" : "-", PMU.getDC5Voltage());
  ESP_LOGCONFIG(TAG, "ALDO1: %s Voltage:%u mV", PMU.isEnableALDO1() ? "+" : "-", PMU.getALDO1Voltage());
  ESP_LOGCONFIG(TAG, "ALDO2: %s Voltage:%u mV", PMU.isEnableALDO2() ? "+" : "-", PMU.getALDO2Voltage());
  ESP_LOGCONFIG(TAG, "ALDO3: %s Voltage:%u mV", PMU.isEnableALDO3() ? "+" : "-", PMU.getALDO3Voltage());
  ESP_LOGCONFIG(TAG, "ALDO4: %s Voltage:%u mV", PMU.isEnableALDO4() ? "+" : "-", PMU.getALDO4Voltage());
  ESP_LOGCONFIG(TAG, "BLDO1: %s Voltage:%u mV", PMU.isEnableBLDO1() ? "+" : "-", PMU.getBLDO1Voltage());
  ESP_LOGCONFIG(TAG, "BLDO2: %s Voltage:%u mV", PMU.isEnableBLDO2() ? "+" : "-", PMU.getBLDO2Voltage());
  ESP_LOGCONFIG(TAG, "CPUSLDO: %s Voltage:%u mV", PMU.isEnableCPUSLDO() ? "+" : "-", PMU.getCPUSLDOVoltage());

  // Power key behavior
  PMU.setPowerKeyPressOffTime(XPOWERS_POWEROFF_4S);
  uint8_t opt = PMU.getPowerKeyPressOffTime();
  switch (opt) {
    case XPOWERS_POWEROFF_4S:  ESP_LOGCONFIG(TAG, "PowerKeyPressOffTime: 4 Second"); break;
    case XPOWERS_POWEROFF_6S:  ESP_LOGCONFIG(TAG, "PowerKeyPressOffTime: 6 Second"); break;
    case XPOWERS_POWEROFF_8S:  ESP_LOGCONFIG(TAG, "PowerKeyPressOffTime: 8 Second"); break;
    case XPOWERS_POWEROFF_10S: ESP_LOGCONFIG(TAG, "PowerKeyPressOffTime: 10 Second"); break;
    default: break;
  }

  PMU.setPowerKeyPressOnTime(XPOWERS_POWERON_128MS);
  opt = PMU.getPowerKeyPressOnTime();
  switch (opt) {
    case XPOWERS_POWERON_128MS: ESP_LOGCONFIG(TAG, "PowerKeyPressOnTime: 128 Ms"); break;
    case XPOWERS_POWERON_512MS: ESP_LOGCONFIG(TAG, "PowerKeyPressOnTime: 512 Ms"); break;
    case XPOWERS_POWERON_1S:    ESP_LOGCONFIG(TAG, "PowerKeyPressOnTime: 1 Second"); break;
    case XPOWERS_POWERON_2S:    ESP_LOGCONFIG(TAG, "PowerKeyPressOnTime: 2 Second"); break;
    default: break;
  }

  // Power-down protections (names differ across XPowersLib versions; keep v1 names)
  bool en;
  en = PMU.getDCHighVoltagePowerDownEn();
  ESP_LOGCONFIG(TAG, "getDCHighVoltagePowerDownEn: %s", en ? "ENABLE" : "DISABLE");
  en = PMU.getDC1LowVoltagePowerDownEn();
  ESP_LOGCONFIG(TAG, "getDC1LowVoltagePowerDownEn: %s", en ? "ENABLE" : "DISABLE");
  en = PMU.getDC2LowVoltagePowerDownEn();
  ESP_LOGCONFIG(TAG, "getDC2LowVoltagePowerDownEn: %s", en ? "ENABLE" : "DISABLE");
  en = PMU.getDC3LowVoltagePowerDownEn();
  ESP_LOGCONFIG(TAG, "getDC3LowVoltagePowerDownEn: %s", en ? "ENABLE" : "DISABLE");
  en = PMU.getDC4LowVoltagePowerDownEn();
  ESP_LOGCONFIG(TAG, "getDC4LowVoltagePowerDownEn: %s", en ? "ENABLE" : "DISABLE");
  en = PMU.getDC5LowVoltagePowerDownEn();
  ESP_LOGCONFIG(TAG, "getDC5LowVoltagePowerDownEn: %s", en ? "ENABLE" : "DISABLE");

  // Board without battery temp sensor -> disable TS measure
  PMU.disableTSPinMeasure();
  PMU.enableTemperatureMeasure();

  // Enable ADC channels we need
  PMU.enableBattDetection();
  PMU.enableVbusVoltageMeasure();
  PMU.enableBattVoltageMeasure();
  PMU.enableSystemVoltageMeasure();

  // Charging LED mode (off by default; you can change later)
  PMU.setChargingLedMode(XPOWERS_CHG_LED_OFF);

  // IRQ pin
  pinMode(pmu_irq_pin, INPUT_PULLUP);
  attachInterrupt(pmu_irq_pin, setFlag, FALLING);

  // IRQ setup
  PMU.disableIRQ(XPOWERS_AXP2101_ALL_IRQ);
  PMU.clearIrqStatus();
  PMU.enableIRQ(
      XPOWERS_AXP2101_BAT_INSERT_IRQ | XPOWERS_AXP2101_BAT_REMOVE_IRQ |
      XPOWERS_AXP2101_VBUS_INSERT_IRQ | XPOWERS_AXP2101_VBUS_REMOVE_IRQ |
      XPOWERS_AXP2101_PKEY_SHORT_IRQ | XPOWERS_AXP2101_PKEY_LONG_IRQ |
      XPOWERS_AXP2101_BAT_CHG_DONE_IRQ | XPOWERS_AXP2101_BAT_CHG_START_IRQ);

  // Charger params
  PMU.setPrechargeCurr(XPOWERS_AXP2101_PRECHARGE_50MA);
  PMU.setChargerConstantCurr(XPOWERS_AXP2101_CHG_CUR_200MA);
  PMU.setChargerTerminationCurr(XPOWERS_AXP2101_CHG_ITERM_25MA);
  PMU.setChargeTargetVoltage(XPOWERS_AXP2101_CHG_VOL_4V1);

  // Watchdog
  PMU.setWatchdogConfig(XPOWERS_AXP2101_WDT_IRQ_TO_PIN);
  PMU.setWatchdogTimeout(XPOWERS_AXP2101_WDT_TIMEOUT_4S);
  PMU.enableWatchdog();

  // Button battery charge
  PMU.enableButtonBatteryCharge();
  PMU.setButtonBatteryChargeVoltage(3300);

  // Apply initial brightness setting (also ensures BLDO1 is on/off as needed)
  this->curr_brightness_ = -1.0f;
  this->UpdateBrightness();
}

void AXP2101Component::dump_config() {
  ESP_LOGCONFIG(TAG, "AXP2101:");
  LOG_I2C_DEVICE(this);
  LOG_SENSOR("  ", "Battery Voltage", this->batteryvoltage_sensor_);
  LOG_SENSOR("  ", "Battery Level", this->batterylevel_sensor_);
  LOG_BINARY_SENSOR("  ", "Battery Charging", this->batterycharging_bsensor_);
}

float AXP2101Component::get_setup_priority() const { return setup_priority::DATA; }

void AXP2101Component::update() {
  // Read once, publish what is configured
  const float vbat_mv = PMU.getBattVoltage();     // XPowersLib typically returns mV
  const float vbat_v  = vbat_mv / 1000.0f;

  if (this->batteryvoltage_sensor_ != nullptr) {
    ESP_LOGD(TAG, "Got Battery Voltage=%f mV", vbat_mv);
    this->batteryvoltage_sensor_->publish_state(vbat_v);
  }

  if (this->batterylevel_sensor_ != nullptr) {
    float batterylevel = NAN;
    if (PMU.isBatteryConnect()) {
      batterylevel = PMU.getBatteryPercent();
    } else {
      // fallback approximation if no battery connected
      batterylevel = 100.0f * ((vbat_v - 3.0f) / (4.1f - 3.0f));
    }

    // Clamp
    if (batterylevel > 100.0f) batterylevel = 100.0f;
    if (batterylevel < 0.0f) batterylevel = 0.0f;

    ESP_LOGD(TAG, "Got Battery Level=%f", batterylevel);
    this->batterylevel_sensor_->publish_state(batterylevel);
  }

  if (this->batterycharging_bsensor_ != nullptr) {
    const bool vcharging = PMU.isCharging();
    ESP_LOGD(TAG, "Got Battery Charging=%s", vcharging ? "true" : "false");
    this->batterycharging_bsensor_->publish_state(vcharging);
  }

  UpdateBrightness();
}

// --- Low-level helpers using ESPHome I2CDevice ---
void AXP2101Component::Write1Byte(uint8_t Addr, uint8_t Data) { this->write_byte(Addr, Data); }

uint8_t AXP2101Component::Read8bit(uint8_t Addr) {
  uint8_t data;
  this->read_byte(Addr, &data);
  return data;
}

void AXP2101Component::ReadBuff(uint8_t Addr, uint8_t Size, uint8_t *Buff) { this->read_bytes(Addr, Buff, Size); }

// --- Backlight control for Core2 v1.1 (AXP2101 BLDO1) ---
void AXP2101Component::UpdateBrightness() {
  // brightness_ is 0.0..1.0
  if (brightness_ == curr_brightness_) return;

  ESP_LOGD(TAG, "Brightness=%f (Curr: %f)", brightness_, curr_brightness_);
  curr_brightness_ = brightness_;

  if (brightness_ <= 0.0f) {
    ESP_LOGD(TAG, "Brightness is zero -> disable BLDO1");
    PMU.disableBLDO1();
    return;
  }

  // Ensure BLDO1 is on
  PMU.enableBLDO1();

  // BLDO1 voltage cfg is register 0x96: 0.5V + N*0.1V, N=0..31
  // 3.3V => N = 28 (0x1C)
  // To make the slider feel usable, map 0..1 to a visible subrange.
  const uint8_t min_vis_step = 20;  // ~2.5V
  const uint8_t max_step     = 31;  // ~3.6V (but clamp later)
  uint8_t step = static_cast<uint8_t>(brightness_ * (max_step - min_vis_step) + 0.5f) + min_vis_step;
  if (step > max_step) step = max_step;

  // Keep upper bits, change only [4:0]
  const uint8_t reg96 = (Read8bit(0x96) & 0xE0) | (step & 0x1F);

  ESP_LOGD(TAG, "Setting BLDO1 step=%u (reg96=0x%02X)", step, reg96);
  Write1Byte(0x96, reg96);

  // Ensure BLDO1 enable bit is set (REG 0x90 bit4) – some boards/boots need this.
  uint8_t reg90 = Read8bit(0x90);
  if ((reg90 & 0x10) == 0) {
    reg90 |= 0x10;
    ESP_LOGD(TAG, "Enabling BLDO1 via REG90 -> 0x%02X", reg90);
    Write1Byte(0x90, reg90);
  }
}

// --- Sleep helpers (kept from your bases) ---
void AXP2101Component::SetSleep() {
  Write1Byte(0x31, Read8bit(0x31) | (1 << 3));     // power off voltage
  Write1Byte(0x90, Read8bit(0x90) | 0x07);         // GPIO1 floating
  Write1Byte(0x82, 0x00);                          // disable ADCs
  Write1Byte(0x12, Read8bit(0x12) & 0xA1);         // disable all outputs but DCDC1
}

void AXP2101Component::DeepSleep(uint64_t time_in_us) {
  SetSleep();
  esp_sleep_enable_ext0_wakeup((gpio_num_t)39, 0 /* LOW */);  // keep v2 pin
  if (time_in_us > 0) {
    esp_sleep_enable_timer_wakeup(time_in_us);
  } else {
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
  }
  (time_in_us == 0) ? esp_deep_sleep_start() : esp_deep_sleep(time_in_us);
}

void AXP2101Component::LightSleep(uint64_t time_in_us) {
  if (time_in_us > 0) {
    esp_sleep_enable_timer_wakeup(time_in_us);
  } else {
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
  }
  esp_light_sleep_start();
}

// --- Startup reason helper ---
std::string AXP2101Component::GetStartupReason() {
  esp_reset_reason_t reset_reason = ::esp_reset_reason();

  if (reset_reason == ESP_RST_DEEPSLEEP) {
    esp_sleep_source_t wake_reason = esp_sleep_get_wakeup_cause();
    if (wake_reason == ESP_SLEEP_WAKEUP_EXT0) return "ESP_SLEEP_WAKEUP_EXT0";
    if (wake_reason == ESP_SLEEP_WAKEUP_EXT1) return "ESP_SLEEP_WAKEUP_EXT1";
    if (wake_reason == ESP_SLEEP_WAKEUP_TIMER) return "ESP_SLEEP_WAKEUP_TIMER";
    if (wake_reason == ESP_SLEEP_WAKEUP_TOUCHPAD) return "ESP_SLEEP_WAKEUP_TOUCHPAD";
    if (wake_reason == ESP_SLEEP_WAKEUP_ULP) return "ESP_SLEEP_WAKEUP_ULP";
    if (wake_reason == ESP_SLEEP_WAKEUP_GPIO) return "ESP_SLEEP_WAKEUP_GPIO";
    if (wake_reason == ESP_SLEEP_WAKEUP_UART) return "ESP_SLEEP_WAKEUP_UART";
    return std::string{"WAKEUP_UNKNOWN_REASON"};
  }

  if (reset_reason == ESP_RST_UNKNOWN) return "ESP_RST_UNKNOWN";
  if (reset_reason == ESP_RST_POWERON) return "ESP_RST_POWERON";
  if (reset_reason == ESP_RST_SW) return "ESP_RST_SW";
  if (reset_reason == ESP_RST_PANIC) return "ESP_RST_PANIC";
  if (reset_reason == ESP_RST_INT_WDT) return "ESP_RST_INT_WDT";
  if (reset_reason == ESP_RST_TASK_WDT) return "ESP_RST_TASK_WDT";
  if (reset_reason == ESP_RST_WDT) return "ESP_RST_WDT";
  if (reset_reason == ESP_RST_BROWNOUT) return "ESP_RST_BROWNOUT";
  if (reset_reason == ESP_RST_SDIO) return "ESP_RST_SDIO";
  return std::string{"RESET_UNKNOWN_REASON"};
}

}  // namespace axp2101
}  // namespace esphome
