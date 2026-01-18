#ifndef __AXP2101_H__
#define __AXP2101_H__

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"

namespace esphome {
namespace axp2101 {

enum AXP2101Model {
  AXP2101_M5CORE2,
};

class AXP2101Component : public PollingComponent, public i2c::I2CDevice {
 public:
  void set_batteryvoltage_sensor(sensor::Sensor *s) { batteryvoltage_sensor_ = s; }
  void set_batterylevel_sensor(sensor::Sensor *s) { batterylevel_sensor_ = s; }
  void set_batterycharging_bsensor(binary_sensor::BinarySensor *s) { batterycharging_bsensor_ = s; }

  void set_brightness(float brightness) { brightness_ = brightness; }
  void set_model(AXP2101Model model) { model_ = model; }

  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override;
  void update() override;

 protected:
  // === Helpers: I2C register access via ESPHome I2CDevice ===
  bool read_u8_(uint8_t reg, uint8_t &val);
  bool write_u8_(uint8_t reg, uint8_t val);
  bool read_buf_(uint8_t reg, uint8_t *buf, size_t len);

  // === Backlight/BLDO1 handling (Core2 v1.1) ===
  void ensure_bldo1_enabled_();
  void set_bldo1_voltage_step_(uint8_t step);  // step: 0..31 -> 0.5V + step*0.1V
  void update_brightness_();

  // === Battery ===
  bool read_batt_voltage_v_(float &volts);
  float estimate_battery_percent_(float v);

  // Sensors
  sensor::Sensor *batteryvoltage_sensor_{nullptr};
  sensor::Sensor *batterylevel_sensor_{nullptr};
  binary_sensor::BinarySensor *batterycharging_bsensor_{nullptr};

  // Config/state
  float brightness_{1.0f};        // 0..1 from cv.percentage
  float curr_brightness_{-1.0f};  // last applied
  AXP2101Model model_{AXP2101_M5CORE2};
};

}  // namespace axp2101
}  // namespace esphome

#endif
