#pragma once
#include <Arduino.h>
#include <Wire.h>

// 簡易 CST816S 觸控（I2C addr 0x15）
class CST816S {
 public:
  bool beginAutodetect(uint16_t w, uint16_t h) {
    _w = w;
    _h = h;
    // 常見 I2C 腳位組合（SDA,SCL）
    const int cand[][3] = {
        {0, 4, 5},  // I2C0 GP4/GP5
        {0, 6, 7},  // I2C0 GP6/GP7（不少 Waveshare 板用這組）
        {1, 2, 3},  // I2C1 GP2/GP3
    };
    for (auto& c : cand) {
      TwoWire* bus = (c[0] == 0) ? &Wire : &Wire1;
      bus->setSDA(c[1]);
      bus->setSCL(c[2]);
      bus->begin();
      if (probe(*bus, 0x15)) {  // 0x15 是 CST816S 常見位址
        _wire = bus;
        _addr = 0x15;
        _ok = true;
        // 軟復位（有些韌體需要）
        writeReg(0xEE, 0x01);
        delay(5);
        return true;
      }
    }
    _ok = false;
    return false;
  }

  // 讀觸控位置，回傳是否有按壓
  bool getTouch(uint16_t* x, uint16_t* y) {
    if (!_ok)
      return false;

    // 讀 0x01..0x06：常見的資料排列（不同韌體可能略有差）
    uint8_t buf[7] = {0};
    if (!readRegs(0x01, buf, 7))
      return false;

    // buf[1] = finger num
    if ((buf[1] & 0x0F) == 0)
      return false;

    // 座標常見拆法（高位低 4bits + 低位 8bits）
    // 參考了多數 CST816S 範例（如若 XY 顛倒，對調即可）
    uint16_t rx = ((buf[3] & 0x0F) << 8) | buf[4];
    uint16_t ry = ((buf[5] & 0x0F) << 8) | buf[6];

    // 限幅
    if (rx >= _w)
      rx = _w - 1;
    if (ry >= _h)
      ry = _h - 1;

    *x = rx;
    *y = ry;
    return true;
  }

 private:
  TwoWire* _wire = nullptr;
  uint8_t _addr = 0x15;
  bool _ok = false;
  uint16_t _w = 240, _h = 240;

  bool probe(TwoWire& w, uint8_t a) {
    w.beginTransmission(a);
    return (w.endTransmission() == 0);
  }
  bool writeReg(uint8_t reg, uint8_t val) {
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    _wire->write(val);
    return _wire->endTransmission() == 0;
  }
  bool readRegs(uint8_t reg, uint8_t* dst, size_t n) {
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    if (_wire->endTransmission(false) != 0)
      return false;
    if (_wire->requestFrom((int)_addr, (int)n) != (int)n)
      return false;
    for (size_t i = 0; i < n; ++i)
      dst[i] = _wire->read();
    return true;
  }
};
