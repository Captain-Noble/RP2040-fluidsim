#pragma once

#include "LowPowerCommon.h"
#include "drivers/rp2040/pico_sleep.h"
#include "hardware/vreg.h"
#include "vector"

namespace low_power {

class ArduinoLowPowerRP2040;
static ArduinoLowPowerRP2040* selfArduinoLowPowerRP2040 = nullptr;

/**
 * @brief Low Power Management for RP2040: In lightSleep we just reduce the
 * processor system clock and voltage. In deepSleep we actually set the
 * processor as dormant, but we must choose between the wakeup by a pin or a
 * timer based wakeup. Only one single wakeup gpio pin is supported!
 *
 * @author Phil Schatzmann
 * for details see
 * -
 * https://github.com/raspberrypi/pico-extras/tree/master/src/rp2_common/pico_sleep
 * - https://github.com/matthias-bs/arduino-pico-sleep/tree/main/src
 * - https://ghubcoder.github.io/posts/awaking-the-pico/
 *
 */

class ArduinoLowPowerRP2040 : public ArduinoLowPowerCommon {
 public:
  ArduinoLowPowerRP2040() { selfArduinoLowPowerRP2040 = this; }

  bool isProcessingOnSleep(sleep_mode_enum_t sleep_mode) {
    bool result = false;
    switch (sleep_mode) {
      case sleep_mode_enum_t::modemSleep:
        result = true;
        break;
      case sleep_mode_enum_t::noSleep:
        result = true;
        break;
      case sleep_mode_enum_t::lightSleep:
        result = true;
        break;
      case sleep_mode_enum_t::deepSleep:
        result = false;
        break;
    }
    return result;
  }

  /// sets processor into sleep mode
  bool sleep(void) override {
    bool rc = false;
    switch (sleep_mode) {
      case sleep_mode_enum_t::lightSleep: {
        int idx = 0;
        for (auto pin : wakeup_pins) {
          attachInterrupt(digitalPinToInterrupt(pin.pin), interrupt_cb,
                          toMode(pin.change_type));
          is_wait_for_pin = true;
        }
        if (is_wait_for_pin) {
          light_sleep_begin();
          while (is_wait_for_pin)
            delay(timer_update_delay);
          light_sleep_end();
        } else {
          light_sleep();
        }
      } break;

      case sleep_mode_enum_t::deepSleep: {
        // use wakup pins
        if (wakeup_pins.size() >= 0) {
          if (wakeup_pins.size() > 1)
            return false;
          if (wakeup_pins[0].change_type == pin_change_t::on_high) {
            sleep_goto_dormant_until_edge_high(wakeup_pins[0].pin);
          } else {
            sleep_goto_dormant_until_edge_low(wakeup_pins[0].pin);
          }
        } else if (sleep_time_us > 0) {
          // use time to sleep
          sleep_goto_sleep_for(sleep_time_us / 1000, timer_cb);
          if (is_restart)
            rp2040.reboot();

        } else {
          // no wakeup !
          processor_deep_sleep();
          if (is_restart)
            rp2040.reboot();
        }
        return true;
      }

      case sleep_mode_enum_t::modemSleep:
        delay(sleep_time_us / 1000);
        return true;

      case sleep_mode_enum_t::noSleep:
        delay(sleep_time_us / 1000);
        light_sleep_end();
        return true;
    }

    return false;
  }

  bool setSleepTime(uint32_t time, time_unit_t time_unit_type) override {
    sleep_time_us = (toUs(time, time_unit_type));
    return sleep_mode == sleep_mode_enum_t::lightSleep ||
           wakeup_pins.size() == 0;
  }

  bool addWakeupPin(int pin, pin_change_t change_type) override {
    PinChangeDef pin_change_def{pin, change_type};
    wakeup_pins.push_back(pin_change_def);
    return sleep_mode == sleep_mode_enum_t::lightSleep || sleep_time_us == 0;
  }

  void clear() {
    ArduinoLowPowerCommon::clear();
    sleep_time_us = 0;
    is_wait_for_pin = false;
    wakeup_pins.clear();
  }

  /// We force a restart after we wake up from sleep
  void setRestart(bool flag) { is_restart = flag; }

 protected:
  struct PinChangeDef {
    int pin;
    pin_change_t change_type;
    PinChangeDef(int p, pin_change_t ct) {
      pin = p;
      change_type = ct;
    }
  };
  std::vector<PinChangeDef> wakeup_pins;
  uint64_t sleep_time_us = 0;
  bool is_wait_for_pin = false;
  bool is_restart = false;
  int timer_update_delay = 2;

  static void timer_cb(unsigned int) {}

  static void interrupt_cb() {
    selfArduinoLowPowerRP2040->is_wait_for_pin = false;
    if (selfArduinoLowPowerRP2040->is_restart)
      rp2040.reboot();
  }

  PinStatus toMode(pin_change_t ct) {
    switch (ct) {
      case pin_change_t::on_high:
        return RISING;
      case pin_change_t::on_low:
        return FALLING;
      default:
        return CHANGE;
    }
  }

  void light_sleep() {
    light_sleep_begin();
    delay(sleep_time_us / 1000);
    light_sleep_end();
  }

  void light_sleep_begin() {
    delay(timer_update_delay);
    if (!set_sys_clock_khz(18000, false)) {
      Serial.println("sleep clock hz is not correct!");
    }
    // Set System clock to 10 MHz,
    // sys_clock_khz(10000, true); did not work for me
    delay(timer_update_delay);
    vreg_set_voltage(VREG_VOLTAGE_0_95);  // 0.85V did not work, 0.95V
    //                                       // seetime_unit_t::ms pretty stable
  }
  void light_sleep_end() {
    if (is_restart)
      rp2040.reboot();
    vreg_set_voltage(
        VREG_VOLTAGE_DEFAULT);  // corresponds to 1.10V, not sure if
                                // that is really required for 48 MHz
    if (!set_sys_clock_khz(240 * 1000, false)) {
      Serial.println("awake clock hz is not correct!");
    }  // Set System clock to 10 MHz,
    delay(timer_update_delay);
  }

  inline void sleep_goto_dormant_until_edge_high(uint gpio_pin) {
    sleep_goto_dormant_until_pin(gpio_pin, true, true);
  }
  inline void sleep_goto_dormant_until_edge_low(uint gpio_pin) {
    sleep_goto_dormant_until_pin(gpio_pin, true, false);
  }
};

static ArduinoLowPowerRP2040 LowPower;

}  // namespace low_power