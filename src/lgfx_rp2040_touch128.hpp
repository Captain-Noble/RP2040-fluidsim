// lgfx_rp2040_touch128.hpp
#pragma once
#include <LovyanGFX.hpp>

// ======== Waveshare RP2040 Touch LCD 1.28 你的接線 ========
// LCD   -> Pico
// VCC -> 3.3V, GND -> GND
// MISO -> 12, MOSI -> 11, SCLK -> 10
// LCD_CS -> 9, LCD_DC -> 14, LCD_RST -> 8, LCD_BL -> 15
// Touch -> Pico
// TP_SDA -> 6, TP_SCL -> 7, TP_INT -> 16, TP_RST -> 17  (I2C addr 0x15)

#define LCD_SPI_PORT 1
#define LCD_PIN_SCLK 10
#define LCD_PIN_MOSI 11
#define LCD_PIN_MISO 12
#define LCD_PIN_CS 9
#define LCD_PIN_DC 14
#define LCD_PIN_RST 8
#define LCD_PIN_BL 15

#define TOUCH_I2C_PORT 1
#define TOUCH_PIN_SDA 6
#define TOUCH_PIN_SCL 7
#define TOUCH_PIN_INT 16
#define TOUCH_PIN_RST 17
#define TOUCH_ADDR 0x15

class LGFX_RP2040_Touch128 : public lgfx::LGFX_Device {
 public:
  LGFX_RP2040_Touch128() {
    // --- SPI Bus ---
    {
      auto cfg = _bus.config();
      cfg.spi_host = LCD_SPI_PORT;  // RP2040: 0 或 1
      cfg.spi_mode = 0;
      cfg.freq_write = 60000000;  // 不穩就降到 40000000
      cfg.freq_read = 16000000;
      cfg.pin_sclk = LCD_PIN_SCLK;
      cfg.pin_mosi = LCD_PIN_MOSI;
      cfg.pin_miso = LCD_PIN_MISO;
      cfg.pin_dc = LCD_PIN_DC;
      _bus.config(cfg);
      _panel.setBus(&_bus);
    }

    // --- 面板 (GC9A01) ---
    {
      auto cfg = _panel.config();
      cfg.pin_cs = LCD_PIN_CS;
      cfg.pin_rst = LCD_PIN_RST;
      cfg.pin_busy = -1;
      cfg.memory_width = 240;
      cfg.memory_height = 240;
      cfg.panel_width = 240;
      cfg.panel_height = 240;
      cfg.offset_x = 0;
      cfg.offset_y = 0;
      cfg.invert = true;  // GC9A01 多半要 true；若畫面反色可改 false 試試
      cfg.readable = false;
      cfg.dummy_read_pixel = 16;
      cfg.bus_shared = false;
      _panel.config(cfg);
    }

    // --- 背光 (PWM) ---
    {
      auto cfg = _light.config();
      cfg.pin_bl = LCD_PIN_BL;  // ★ 你的 BL=15
      cfg.invert = false;
      cfg.freq = 48000;
      cfg.pwm_channel = 0;
      _light.config(cfg);
      _panel.setLight(&_light);
    }

    // --- 觸控 (CST816S / I2C) ---
    {
      auto cfg = _touch.config();
      cfg.i2c_port = TOUCH_I2C_PORT;  // I2C1
      cfg.pin_sda = TOUCH_PIN_SDA;
      cfg.pin_scl = TOUCH_PIN_SCL;
      cfg.pin_int = TOUCH_PIN_INT;  // 沒接可設 -1
      cfg.pin_rst = TOUCH_PIN_RST;  // 沒接可設 -1
      cfg.i2c_addr = TOUCH_ADDR;    // 0x15
      cfg.freq = 400000;
      cfg.x_max = 240;
      cfg.y_max = 240;
      _touch.config(cfg);
    }

    setPanel(&_panel);
  }

  // 提供 main.cpp 使用：像舊的 display.getTouch(&x,&y)
  bool getTouch(uint16_t* x, uint16_t* y) {
    lgfx::touch_point_t tp;
    if (_touch.getTouchRaw(&tp, 1)) {  // ★ 需要傳 count=1
      // 依當前旋轉把座標轉到你的畫面座標
      const int W = 240, H = 240;
      switch (getRotation() & 3) {
        case 0:
          *x = tp.x;
          *y = tp.y;
          break;
        case 1:
          *x = W - 1 - tp.y;
          *y = tp.x;
          break;
        case 2:
          *x = W - 1 - tp.x;
          *y = H - 1 - tp.y;
          break;
        case 3:
          *x = tp.y;
          *y = H - 1 - tp.x;
          break;
      }
      return true;
    }
    return false;
  }

 private:
  lgfx::Panel_GC9A01 _panel;
  lgfx::Bus_SPI _bus;
  lgfx::Light_PWM _light;
  lgfx::Touch_CST816S _touch;
};
