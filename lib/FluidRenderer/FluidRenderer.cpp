#include "FluidRenderer.hpp"

// 16-bit 565 颜色线性插值
uint16_t FluidRenderer::lerp565(uint16_t c1, uint16_t c2, float t) const {
  // clamp t to [0,1]
  if (t < 0.0f)
    t = 0.0f;
  if (t > 1.0f)
    t = 1.0f;

  auto lerp = [t](int a, int b, int bits) -> uint16_t {
    const int mask = (1 << bits) - 1;
    float v = a + (b - a) * t;  // ① 浮点插值
    int iv = (int)roundf(v);    // ② 四舍五入取整
    iv = iv & mask;             // ③ 掩码裁剪
    return (uint16_t)iv;
  };

  uint16_t r = lerp((c1 >> 11) & 0x1F, (c2 >> 11) & 0x1F, 5);  // 5 bits
  uint16_t g = lerp((c1 >> 5) & 0x3F, (c2 >> 5) & 0x3F, 6);    // 6 bits
  uint16_t b = lerp(c1 & 0x1F, c2 & 0x1F, 5);                  // 5 bits

  return (r << 11) | (g << 5) | b;
}

// ------------------ 公共接口 ------------------

void FluidRenderer::render(Mode mode) {
  m_disp->startWrite();
  switch (mode) {
    case BALLS:
      renderBalls();
      break;
    case GRID:
      renderGrid();
      break;
    case PARTIAL_GRID:
      renderPartialGrid();
      break;
  }
  m_disp->endWrite();
}

void FluidRenderer::renderBalls() {
  // 1. 背景
  m_disp->fillScreen(m_disp->color565(0, 0, 0));

  // 2. 容器边框
  m_disp->drawRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, TFT_WHITE);

  // 3. 画每个粒子
  const Particle* P = m_sim->data();
  const int N = m_sim->particleCount();
  const int radius = (int)(PIXEL_PER_CELL * PARTICLE_RADIUS);

  for (int i = 0; i < N; ++i) {
    int sx = (int)(P[i].x * PIXEL_PER_CELL);
    int sy = (int)(P[i].y * PIXEL_PER_CELL);

    // 速度映射到颜色
    float speed = sqrtf(P[i].vx * P[i].vx + P[i].vy * P[i].vy);
    float t = constrain(speed / 8.0f, 0.0f, 1.0f);
    uint16_t c = lerp565(m_ballBase, TFT_WHITE, t);

    m_disp->fillCircle(sx, sy, radius, c);
    m_disp->drawCircle(sx, sy, radius, TFT_NAVY);  // 细圈
  }
}

void FluidRenderer::renderGrid() {
  // 0. 颜色表（可再做成成员变量）
  const uint16_t emptyCol = m_disp->color565(5, 5, 20);      // 深蓝：空气
  const uint16_t liquidCol = m_gridFluid;                    // 现有液体色
  const uint16_t foamCol = m_disp->color565(230, 230, 230);  // 近白：泡沫

  // 1. 先清屏
  m_disp->fillScreen(emptyCol);

  // 2. 逐格绘制
  for (int gx = 0; gx < LOGICAL_GRID_SIZE; ++gx) {
    for (int gy = 0; gy < LOGICAL_GRID_SIZE; ++gy) {
      int px = gx * PIXEL_PER_CELL;
      int py = gy * PIXEL_PER_CELL;

      uint16_t color;

      if (m_sim->isSolid(gx, gy)) {
        color = m_gridSolid;  // 墙
      } else {
        // 读取当前帧流体状态
        FluidType ft = m_sim->m_currFluid[gx * LOGICAL_GRID_SIZE + gy];
        switch (ft) {
          case FLUID_LIQUID:
            color = liquidCol;
            break;
          case FLUID_FOAM:
            color = foamCol;
            break;
          default:
            color = emptyCol;
            break;  // FLUID_EMPTY
        }
      }

      m_disp->fillRect(px, py, PIXEL_PER_CELL, PIXEL_PER_CELL, color);

      // （可选）描边
      m_disp->drawRect(px, py, PIXEL_PER_CELL, PIXEL_PER_CELL,
                       m_disp->color565(10, 10, 20));
    }
  }
}

void FluidRenderer::renderPartialGrid() {
  const int* ids = m_sim->changedIndices();
  const int cnt = m_sim->changedCount();

  // 统计并打印
  //   Serial.printf("Changed cells this frame: %d\n", cnt);

  for (int n = 0; n < cnt; ++n) {
    int idx = ids[n];
    int gx = idx / LOGICAL_GRID_SIZE;
    int gy = idx % LOGICAL_GRID_SIZE;

    if (m_sim->isSolid(gx, gy))
      continue;  // 固体不用重画

    // 像素坐标
    int px = gx * PIXEL_PER_CELL;
    int py = gy * PIXEL_PER_CELL;

    // 颜色
    uint16_t color;
    switch (m_sim->m_currFluid[idx]) {
      case FLUID_LIQUID:
        color = m_gridFluid;
        break;
      case FLUID_FOAM:
        color = m_gridFoam;
        break;
      default:
        color = m_disp->color565(5, 5, 20);
        break;
    }

    // 绘制
    m_disp->fillRect(px, py, PIXEL_PER_CELL, PIXEL_PER_CELL, color);
    m_disp->drawRect(px, py, PIXEL_PER_CELL, PIXEL_PER_CELL,
                     m_disp->color565(10, 10, 20));
  }
}
