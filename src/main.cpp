#include <Wire.h>
#include <stdint.h>
#include <cmath>  // 数学库，用于计算平方根等
#include "lgfx_gc9a01.hpp"
#include "qmi8658c.hpp"  // 加速度计头文件

// 全局变量
static LGFX_GC9A01 display;  // 显示屏
static QMI8658C imu;         // 加速度计

<<<<<<< Updated upstream
// 屏幕逻辑网格定义
#define LOGICAL_GRID_SIZE_X 24             // 逻辑网格宽度
#define LOGICAL_GRID_SIZE_Y 24             // 逻辑网格高度
#define PIXEL_PER_CELL 10                  // 每个逻辑像素的物理像素大小
#define ELASTICITY 0.5f                    // 碰撞弹性系数
#define PARTICLE_RADIUS 0.5f               // 粒子半径，逻辑单位
#define MIN_DIST (2.0f * PARTICLE_RADIUS)  // 最小允许粒子间距
#define MIN_DIST2 (MIN_DIST * MIN_DIST)    // 最小间距平方
=======
// ─────────────── 帧率 / 物理步长设定 ───────────────
static unsigned long lastFrameTime = 0;
static constexpr float TARGET_FPS = 10.0f;
static constexpr float FIXED_DT = 1.0f / TARGET_FPS;
>>>>>>> Stashed changes

// 粒子结构，使用 float 存储位置和速度
struct Particle {
  float x, y;    // 粒子在逻辑网格中的位置
  float vx, vy;  // 粒子速度
};

static Particle particles[NUM_PARTICLES];  // 粒子数组
static float velocityGrid[LOGICAL_GRID_SIZE_X][LOGICAL_GRID_SIZE_Y]
                         [2];  // 网格速度场
static uint8_t gridType[LOGICAL_GRID_SIZE_X][LOGICAL_GRID_SIZE_Y];  // 网格类型
float ax = 0.0f, ay = 0.0f;                                         // 加速度值

// 初始化粒子和网格
void initSimulation() {
  // 初始化粒子
  for (int i = 0; i < NUM_PARTICLES; i++) {
    particles[i].x = random(0, LOGICAL_GRID_SIZE_X);
    particles[i].y = random(0, LOGICAL_GRID_SIZE_Y);
    particles[i].vx = random(-200, 200) / 100.0f;
    particles[i].vy = random(-200, 200) / 100.0f;
  }

  // 初始化网格
  for (int x = 0; x < LOGICAL_GRID_SIZE_X; x++) {
    for (int y = 0; y < LOGICAL_GRID_SIZE_Y; y++) {
      if ((x - LOGICAL_GRID_SIZE_X / 2) * (x - LOGICAL_GRID_SIZE_X / 2) +
              (y - LOGICAL_GRID_SIZE_Y / 2) * (y - LOGICAL_GRID_SIZE_Y / 2) >=
          (LOGICAL_GRID_SIZE_X / 2) * (LOGICAL_GRID_SIZE_X / 2)) {
        gridType[x][y] = SOLID_CELL;  // 圆形边界外为固体
      } else {
        gridType[x][y] = FLUID_CELL;  // 圆形边界内为流体
      }
      velocityGrid[x][y][0] = 0.0f;  // 初始速度为0
      velocityGrid[x][y][1] = 0.0f;
    }
  }
}

// 粒子之间的碰撞检测与处理
void handleParticleCollisions() {
  for (int i = 0; i < NUM_PARTICLES; i++) {
    for (int j = i + 1; j < NUM_PARTICLES; j++) {
      float dx = particles[j].x - particles[i].x;
      float dy = particles[j].y - particles[i].y;
      float dist2 = dx * dx + dy * dy;

      if (dist2 < MIN_DIST2 && dist2 > 0.0f) {
        float dist = sqrt(dist2);
        float overlap = 0.5f * (MIN_DIST - dist) / dist;

        // 调整位置以解决重叠
        dx *= overlap;
        dy *= overlap;
        particles[i].x -= dx;
        particles[i].y -= dy;
        particles[j].x += dx;
        particles[j].y += dy;

        // 调整速度以模拟碰撞
        float relativeVx = particles[j].vx - particles[i].vx;
        float relativeVy = particles[j].vy - particles[i].vy;
        float dotProduct = relativeVx * dx + relativeVy * dy;

        if (dotProduct > 0) {
          float impulse = dotProduct / dist2;
          dx *= impulse * ELASTICITY;
          dy *= impulse * ELASTICITY;

          particles[i].vx += dx;
          particles[i].vy += dy;
          particles[j].vx -= dx;
          particles[j].vy -= dy;
        }
      }
    }
  }
}

// 更新粒子位置
void updateParticles(float dt) {
  for (int i = 0; i < NUM_PARTICLES; i++) {
    // 更新粒子速度，加入加速度计的影响
    particles[i].vx += ax * dt;
    particles[i].vy += ay * dt;

    int gridX = static_cast<int>(particles[i].x);
    int gridY = static_cast<int>(particles[i].y);

    if (gridX < 0 || gridY < 0 || gridX >= LOGICAL_GRID_SIZE_X ||
        gridY >= LOGICAL_GRID_SIZE_Y)
      continue;

    particles[i].vx += velocityGrid[gridX][gridY][0] * dt;
    particles[i].vy += velocityGrid[gridX][gridY][1] * dt;

    // 更新粒子位置
    particles[i].x += particles[i].vx * dt;
    particles[i].y += particles[i].vy * dt;

    // 边界处理
    if (particles[i].x < 0 || particles[i].x >= LOGICAL_GRID_SIZE_X) {
      particles[i].vx *= -ELASTICITY;
      particles[i].x =
          constrain(particles[i].x, 0.0f, LOGICAL_GRID_SIZE_X - 1.0f);
    }
    if (particles[i].y < 0 || particles[i].y >= LOGICAL_GRID_SIZE_Y) {
      particles[i].vy *= -ELASTICITY;
      particles[i].y =
          constrain(particles[i].y, 0.0f, LOGICAL_GRID_SIZE_Y - 1.0f);
    }
  }

  // 处理粒子之间的碰撞
  handleParticleCollisions();
}

// 绘制粒子和网格
void render() {
  display.startWrite();
  display.fillScreen(TFT_BLACK);

  // 绘制粒子
  for (int i = 0; i < NUM_PARTICLES; i++) {
    int16_t screenX = static_cast<int16_t>(particles[i].x * PIXEL_PER_CELL);
    int16_t screenY = static_cast<int16_t>(particles[i].y * PIXEL_PER_CELL);
    display.fillCircle(screenX, screenY, PIXEL_PER_CELL / 2, TFT_BLUE);
  }

  // 绘制网格
  for (int x = 0; x < LOGICAL_GRID_SIZE_X; x++) {
    for (int y = 0; y < LOGICAL_GRID_SIZE_Y; y++) {
      if (gridType[x][y] == SOLID_CELL) {
        display.fillRect(x * PIXEL_PER_CELL, y * PIXEL_PER_CELL, PIXEL_PER_CELL,
                         PIXEL_PER_CELL, TFT_GRAY);
      }
    }
  }

  display.endWrite();
}

// 更新加速度计数据
void updateIMU() {
  float accelX, accelY, accelZ;
  imu.readAccelerometer(&accelX, &accelY, &accelZ);
  ax = accelX * 10.0f;  // 调整为合适的比例
  ay = accelY * 10.0f;
}

// 主函数
void setup() {
  Serial.begin(115200);
  display.begin(PIN_LCD_SCLK, PIN_LCD_MOSI, PIN_LCD_DC, PIN_LCD_CS, PIN_LCD_RST,
                PIN_LCD_BL);

  // 初始化 IMU
  Wire1.setSDA(PIN_IMU_SDA);
  Wire1.setSCL(PIN_IMU_SCL);
  Wire1.setClock(400000);
  Wire1.begin();
  imu.begin(&Wire1, QMI8658C_I2C_ADDRESS_PULLUP);
  imu.configureAcc(QMI8658C::AccScale::ACC_SCALE_4G,
                   QMI8658C::AccODR::ACC_ODR_250HZ,
                   QMI8658C::AccLPF::ACC_LPF_5_32PCT);

  initSimulation();
}

void loop() {
<<<<<<< Updated upstream
  updateIMU();            // 更新加速度计数据
  updateParticles(0.1f);  // 更新粒子
  render();               // 绘制结果
  // delay(40);              // 控制帧率
=======
  unsigned long nowMs = millis();
  float deltaTime = (nowMs - lastFrameTime) * 0.001f;

  // ----------- 固定步长物理 -----------
  static float acc = 0.0f;
  acc += deltaTime;
  acc = min(acc, 0.1f);

  while (acc >= FIXED_DT) {
    unsigned long t0 = micros();
    sim.simulate(FIXED_DT);
    physAccumUs += micros() - t0;
    acc -= FIXED_DT;
  }

  // ----------- 渲染（格子模式） -----------
  unsigned long t0 = micros();
  renderer.render(FluidRenderer::PARTIAL_GRID);
  rendAccumUs += micros() - t0;

  // --- 叠加 FPS 文本 ---
  static int fpsFrames = 0;
  static unsigned long fpsTimer = 0;
  static float fps = 0.0f;

  ++fpsFrames;
  if (nowMs - fpsTimer >= 1000) {
    fps = fpsFrames * 1000.0f / (nowMs - fpsTimer);
    fpsFrames = 0;
    fpsTimer = nowMs;
  }
  display.startWrite();
  display.setTextColor(TFT_WHITE, display.color565(0, 0, 16));
  display.setCursor(5, 5);
  display.printf("FPS: %.1f", fps);
  display.endWrite();

  lastFrameTime = nowMs;
  ++frameCounter;

  // ----------- 每秒串口打印统计 -----------
  if (nowMs - lastPrintMs >= 1000) {
    Serial.printf("[FPS %3u] Physics: %.2f ms  |  Render: %.2f ms\r\n",
                  frameCounter, physAccumUs * 0.001f, rendAccumUs * 0.001f);
    physAccumUs = rendAccumUs = 0;
    frameCounter = 0;
    lastPrintMs = nowMs;
  }
>>>>>>> Stashed changes
}
