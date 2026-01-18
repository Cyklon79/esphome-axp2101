#ifndef __AXP2101_H__
#define __AXP2101_H__

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"

#define XPOWERS_CHIP_AXP2101
#include "XPowersLib.h"

namespace esphome {
namespace axp2101 {

enum AXP2101Model : uint8_t {
  AXP2101_M5CORE2,
};

class AXP2101Component : public PollingComponent, public i2c::I2CDevice {
 public:
  void set_batteryvoltage_sensor(sensor::Sensor *s) { batteryvoltage_sensor_ = s; }
  void set_batterylevel_sensor(sensor::Sensor *s) { batterylevel_sensor_ = s; }
  void set_batterycharging_bsensor(binary_sensor::BinarySensor *s) { batterycharging_bsensor_ = s; }

  // cv.percentage => 0.0..1.0
  void set_brightness(float brightness) { brightness_ = brightness; }

  void set_model(AXP2101Model model) { model_ = model; }

  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override;
  void update() override;

 protected:
  // Optional internal helpers (keep internal until you expose in YAML)
  void set_lcd_enabled_(bool on);
  void set_blue_led_(bool on);
  void set_speaker_enabled_(bool on);

  static std::string GetStartupReason();

  sensor::Sensor *batteryvoltage_sensor_{nullptr};
  sensor::Sensor *batterylevel_sensor_{nullptr};
  binary_sensor::BinarySensor *batterycharging_bsensor_{nullptr};

  float brightness_{1.0f};
  float curr_brightness_{-1.0f};
  AXP2101Model model_{AXP2101_M5CORE2};

  void UpdateBrightness();

  // (reszta helperów I2C / legacy / sleep itd. jeśli naprawdę używasz)
  void Write1Byte(uint8_t addr, uint8_t data);
  uint8_t Read8bit(uint8_t addr);
  void ReadBuff(uint8_t addr, uint8_t size, uint8_t *buf);

  // sleep – jeśli używasz:
  void SetSleep();
  void DeepSleep(uint64_t time_in_us = 0);
  void LightSleep(uint64_t time_in_us = 0);
};

}  // namespace axp2101
}  // namespace esphome

#endif
