#pragma once

// #include <Arduino.h>  // 為了 pinMode / attachInterrupt / RISING 等
#include <vector>
#include "LowPowerCommon.h"
#include "drivers/rp2040/pico_sleep.h"
#include "hardware/vreg.h"

namespace low_power {

class ArduinoLowPowerRP2040;
static ArduinoLowPowerRP2040* selfArduinoLowPowerRP2040 = nullptr;

/**
 * @brief Low Power Management for RP2040
 * - lightSleep：降時脈＋降壓；可用 GPIO 中斷或定時喚醒
 * - deepSleep：進入 dormant；只能擇一：單一 GPIO 邊沿喚醒 或 定時喚醒
 *  （僅支援單一喚醒腳）
 */
class ArduinoLowPowerRP2040 : public ArduinoLowPowerCommon {
 public:
  ArduinoLowPowerRP2040() { selfArduinoLowPowerRP2040 = this; }

  bool isProcessingOnSleep(sleep_mode_enum_t sm) {
    switch (sm) {
      case sleep_mode_enum_t::modemSleep:
      case sleep_mode_enum_t::noSleep:
      case sleep_mode_enum_t::lightSleep:
        return true;
      case sleep_mode_enum_t::deepSleep:
        return false;
    }
    return false;
  }

  /// 讓處理器進入睡眠
  bool sleep(void) override {
    switch (sleep_mode) {
      case sleep_mode_enum_t::lightSleep: {
        // 掛上所有喚醒中斷
        for (auto pin : wakeup_pins) {
          attachInterrupt(pin.pin, interrupt_cb, toMode(pin.change_type));
          is_wait_for_pin = true;
        }

        if (is_wait_for_pin) {
          light_sleep_begin();
          while (is_wait_for_pin)
            delay(timer_update_delay);
          light_sleep_end();

          // 醒來後把剛剛掛的中斷都解除，避免重複觸發
          for (auto pin : wakeup_pins) {
            detachInterrupt(pin.pin);
          }
        } else {
          // 若沒有指定喚醒腳，則用時間喚醒（需先 setSleepTime）
          light_sleep();
        }
        return true;
      } break;

      case sleep_mode_enum_t::deepSleep: {
        // --- 修正：必須用 > 0，否則 size()==0 時會越界 ---
        if (wakeup_pins.size() > 0) {
          if (wakeup_pins.size() > 1)
            return false;  // deep 僅支援單一腳
          if (wakeup_pins[0].change_type == pin_change_t::on_high) {
            sleep_goto_dormant_until_edge_high(wakeup_pins[0].pin);
          } else {
            sleep_goto_dormant_until_edge_low(wakeup_pins[0].pin);
          }
        } else if (sleep_time_us > 0) {
          // 註：這裡除以 1000 代表 API 參數為「毫秒」；若你的 SDK 期望
          // μs，請移除 /1000
          sleep_goto_sleep_for(sleep_time_us / 1000, timer_cb);
          if (is_restart)
            rp2040.reboot();
        } else {
          // 無任何喚醒來源：純 dormant（喚不醒，僅供特殊用途）
          processor_deep_sleep();
          if (is_restart)
            rp2040.reboot();
        }
        return true;
      } break;

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
    sleep_time_us = toUs(time, time_unit_type);
    return sleep_mode == sleep_mode_enum_t::lightSleep ||
           wakeup_pins.size() == 0;
  }

  bool addWakeupPin(int pin, pin_change_t change_type) override {
    PinChangeDef pin_change_def{pin, change_type};
    wakeup_pins.push_back(pin_change_def);
    return sleep_mode == sleep_mode_enum_t::lightSleep || sleep_time_us == 0;
  }

  /// 方便用於「搖晃喚醒」的感測器 INT 腳註冊（activeHigh=true 表 INT 高時觸發）
  bool addShakeWakePin(int pin, bool activeHigh = true) {
    pinMode(pin, activeHigh ? INPUT_PULLDOWN : INPUT_PULLUP);  // 穩定邊沿
    return addWakeupPin(
        pin, activeHigh ? pin_change_t::on_high : pin_change_t::on_low);
  }

  void clear() {
    ArduinoLowPowerCommon::clear();
    sleep_time_us = 0;
    is_wait_for_pin = false;
    wakeup_pins.clear();
  }

  /// 醒來後是否立刻重開機
  void setRestart(bool flag) { is_restart = flag; }

 protected:
  struct PinChangeDef {
    int pin;
    pin_change_t change_type;
    PinChangeDef(int p, pin_change_t ct) : pin(p), change_type(ct) {}
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

  // 回傳型別改成 int，以符合 attachInterrupt 的第三參數
  static PinStatus toMode(pin_change_t ct) {
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
    delay(timer_update_delay);
    vreg_set_voltage(VREG_VOLTAGE_0_95);  // 0.95V 在多數板上穩定
  }

  void light_sleep_end() {
    if (is_restart)
      rp2040.reboot();
    vreg_set_voltage(VREG_VOLTAGE_DEFAULT);  // 約 1.10V
    if (!set_sys_clock_khz(200 * 1000, false)) {
      Serial.println("awake clock hz is not correct!");
    }
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
