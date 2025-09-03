/******************************************************************
 *  main.cpp  ――  Fluid-SIM + 低功耗状态机 (RP2040)
 ******************************************************************/
#include <Wire.h>
#include <math.h>
#include "FluidRenderer.hpp"
#include "LowPowerRP2040.h"  // 你的低功耗類
#include "ParticleSimulation.hpp"
#include "lgfx_gc9a01.hpp"
#include "qmi8658c.hpp"

using namespace low_power;

/* ────── 選擇性：開 DEBUG 看加速度值與單位 ───── */
#define SHAKE_DEBUG 1

/* ────── 硬件/模块 ─────────────────────────── */
static LGFX_GC9A01 display;
static QMI8658C imu;
static ParticleSimulation sim;
static FluidRenderer renderer(&display, &sim);
static ArduinoLowPowerRP2040 lp;

/* ────── 运行参数 ──────────────────────────── */
static constexpr float TARGET_FPS = 30.f;
static constexpr float FIXED_DT = 1.f / TARGET_FPS;

/* ────── 运动检测参数（用 gyro 判靜止） ─────── */
static constexpr float GYRO_EPS = 10.0f;     // dps Δ阈值
static constexpr uint32_t STILL_MS = 30000;  // 靜止多久進省電
static constexpr uint32_t DETECT_MS = 1000;  // 輪詢睡眠時間片

/* ────── 状态机 ────────────────────────────── */
enum class AppState { RUNNING, GO_SLEEP, SLEEP_POLL };
static AppState state = AppState::RUNNING;

/* ────── 陀螺仪監視 ────────────────────────── */
static float prevGx = 0, prevGy = 0;
static uint32_t stillTimer = 0;

/* 若要改用 WoM 中斷喚醒 deep-sleep，定義 IMU INT 腳位 */
// #define PIN_IMU_INT  21

/* ────── 讀 gyro 是否在動 ─────────────────── */
static bool gyroMoving(float& dxdy) {
  float gx, gy, gz;
  if (!imu.readGyroscope(&gx, &gy, &gz))
    return true;  // 讀失敗視為在動
  dxdy = hypotf(gx - prevGx, gy - prevGy);
  prevGx = gx;
  prevGy = gy;
  return dxdy > GYRO_EPS;
}

/* ────── 把加速度餵給渲染器（自動單位→g） ─────
   - 很多 QMI8658 庫回傳「mg」，不是 g！
   - heuristic：
       mag >= 200 → 當 mg，/1000 → g
       20 <= mag < 200 → 當 m/s^2，/9.80665 → g
       其餘 → 已是 g
*/
static void feedRendererAccel() {
  float ax, ay, az;
  if (!imu.readAccelerometer(&ax, &ay, &az))
    return;

  float mag = sqrtf(ax * ax + ay * ay + az * az);
  const float INV_G = 1.0f / 9.80665f;
  const float INV_1K = 1.0f / 1000.0f;

  const char* unit = "g";
  if (mag >= 200.0f) {  // 典型 mg：~1000
    ax *= INV_1K;
    ay *= INV_1K;
    az *= INV_1K;
    unit = "mg->g";
  } else if (mag >= 20.0f) {  // 典型 m/s^2：~9.8
    ax *= INV_G;
    ay *= INV_G;
    az *= INV_G;
    unit = "ms2->g";
  } else {
    unit = "g";
  }

#if SHAKE_DEBUG
  static uint32_t lastPrint = 0;
  uint32_t now = millis();
  if (now - lastPrint > 200) {  // 每 200ms 列一次
    Serial.printf("[ACC %s] ax=%.2f ay=%.2f az=%.2f mag=%.2f\n", unit, ax, ay,
                  az, sqrtf(ax * ax + ay * ay + az * az));
    lastPrint = now;
  }
#endif

  // 餵給 FluidRenderer 做方向用力搖晃檢測 → 換色
  renderer.onAccelSample(ax, ay, az);
}

/* ────── 初始化 ────────────────────────────── */
void setup() {
  Serial.begin(115200);

  // LCD
  display.begin(PIN_LCD_SCLK, PIN_LCD_MOSI, PIN_LCD_DC, PIN_LCD_CS, PIN_LCD_RST,
                PIN_LCD_BL);
  display.setRotation(0);
  display.fillScreen(TFT_BLACK);
  display.setBrightness(255);

  // I2C
  Wire1.setSDA(PIN_IMU_SDA);
  Wire1.setSCL(PIN_IMU_SCL);
  Wire1.setClock(400000);
  Wire1.begin();

  // IMU
  imu.begin(&Wire1, QMI8658C_I2C_ADDRESS_PULLUP);

  // Gyro：你原本這行 OK
  imu.configureGyro(QMI8658C::GyroScale::GYRO_SCALE_256DPS,
                    QMI8658C::GyroODR::GYRO_ODR_250HZ);

  // ★ Accel：把 configureAccel(...) 改成 configureAcc(...)
  //    枚舉也要用 AccScale / AccODR
  imu.configureAcc(QMI8658C::AccScale::ACC_SCALE_2G,
                   QMI8658C::AccODR::ACC_ODR_125HZ);

  // ★ 啟用感測器（必要）
  imu.enable(/*enable_gyro=*/true, /*enable_acc=*/true);

  // （可選）若要 deep-sleep + WoM 喚醒，這裡把 WoM 門檻設嚴一些
  // imu.enableWakeOnMotion(/*mg=*/800, /*samples=*/8, /*activeHigh=*/true);
  // lp.setSleepMode(sleep_mode_enum_t::deepSleep);
  // lp.addShakeWakePin(PIN_IMU_INT, /*activeHigh=*/true);

  // 模擬與渲染
  sim.begin(&imu);
  renderer.setGridSolidColor(TFT_DARKGREY);
  renderer.setGridFluidColor(TFT_BLUE);

  // 低功耗（先用 lightSleep 輪詢）
  lp.setSleepMode(sleep_mode_enum_t::lightSleep);
  lp.setRestart(false);

  stillTimer = millis();
}

/* ────── 主循环 ────────────────────────────── */
void loop() {
  float dG = 0.f;  // 本帧 gyro Δ

  switch (state) {
    case AppState::RUNNING: {
      // 物理 & 渲染
      sim.simulate(FIXED_DT);

      // 餵加速度做搖晃改色
      feedRendererAccel();

      // 畫面
      renderer.render(FluidRenderer::PARTIAL_GRID);

      // 判靜止：夠久就進省電
      bool moving = gyroMoving(dG);
      if (moving) {
        stillTimer = millis();
      } else if (millis() - stillTimer > STILL_MS) {
        state = AppState::GO_SLEEP;
      }
      break;
    }

    case AppState::GO_SLEEP: {
      display.setBrightness(0);  // 關背光
      state = AppState::SLEEP_POLL;
      break;
    }

    case AppState::SLEEP_POLL: {
      // 輕睡一段時間，醒來輪詢是否有動
      lp.setSleepTime(DETECT_MS, time_unit_t::ms);
      lp.sleep();

      bool moving = gyroMoving(dG);
      if (moving) {
        stillTimer = millis();
        state = AppState::RUNNING;
        display.setBrightness(255);  // 開背光
      }
      break;
    }
  }
}
