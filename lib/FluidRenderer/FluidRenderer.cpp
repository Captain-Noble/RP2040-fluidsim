#include "FluidRenderer.hpp"
#include <Arduino.h>
#include <math.h>
#include <string.h>

// 邊框
#define SHOW_CELL_OUTLINE 1
#define SHOW_SCREEN_FRAME 0

// ========== 你的渲染參數 ==========
#define RENDER_PARTICLE_THRESHOLD 3
#define RENDER_RIM_PARTICLE_THRESHOLD 1
#define RENDER_EDGE_SMOOTH_RADIUS 3
#define RENDER_FOAM_SPEED_THRESHOLD 99.0f
#ifndef PARTICLE_RADIUS
#define PARTICLE_RADIUS (1.6f / 240.0f)
#endif

// ========== 搖晃偵測參數（可調） ==========
// ---------------- 搖晃偵測參數（先放鬆，確認能觸發） ----------------
#define SHAKE_USE_HIGHPASS 1         // 1=去掉重力（高通）
#define SHAKE_LP_ALPHA 0.20f         // 低通係數（抓重力）
#define SHAKE_THR_G 0.35f            // ← 門檻先降，之後再調回 0.8
#define SHAKE_SAMPLES_HIT 3          // ← 連續樣本數先降，之後再調回 6~10
#define SHAKE_DOMINANCE_RATIO 1.20f  // ← 主軸優勢比先降，之後再調回 1.4~1.8
#define SHAKE_COOLDOWN_MS 400
#define SHAKE_INVERT_X 0
#define SHAKE_INVERT_Y 0
#define SHAKE_USE_Z 1  // ← 新增：把 Z 軸也納入判定

// ========== 565 線性插值 ==========
uint16_t FluidRenderer::lerp565(uint16_t c1, uint16_t c2, float t) const {
  if (t < 0.0f)
    t = 0.0f;
  if (t > 1.0f)
    t = 1.0f;

  auto lerp = [t](int a, int b, int bits) -> uint16_t {
    const int mask = (1 << bits) - 1;
    float v = a + (b - a) * t;
    int iv = (int)roundf(v);
    iv &= mask;
    return (uint16_t)iv;
  };
  uint16_t r = lerp((c1 >> 11) & 0x1F, (c2 >> 11) & 0x1F, 5);
  uint16_t g = lerp((c1 >> 5) & 0x3F, (c2 >> 5) & 0x3F, 6);
  uint16_t b = lerp(c1 & 0x1F, c2 & 0x1F, 5);
  return (uint16_t)((r << 11) | (g << 5) | b);
}

// ========== 動態顏色表：依搖晃方向套用主題 ==========
void FluidRenderer::applyPaletteForDir(ShakeDir d) {
  switch (d) {
    case SHAKE_LEFT:  // 冷藍
      pal_liquid = lgfx::color565(0, 220, 255);
      pal_foam = lgfx::color565(220, 240, 255);
      pal_rimTrans = lgfx::color565(0, 140, 255);
      pal_rimLight = lgfx::color565(0, 50, 230);
      break;
    case SHAKE_RIGHT:  // 熱紅
      pal_liquid = lgfx::color565(255, 100, 60);
      pal_foam = lgfx::color565(255, 220, 200);
      pal_rimTrans = lgfx::color565(255, 140, 100);
      pal_rimLight = lgfx::color565(220, 60, 40);
      break;
    case SHAKE_UP:  // 綠青
      pal_liquid = lgfx::color565(20, 230, 160);
      pal_foam = lgfx::color565(210, 255, 230);
      pal_rimTrans = lgfx::color565(40, 180, 150);
      pal_rimLight = lgfx::color565(10, 120, 110);
      break;
    case SHAKE_DOWN:  // 紫色
      pal_liquid = lgfx::color565(160, 80, 255);
      pal_foam = lgfx::color565(235, 220, 255);
      pal_rimTrans = lgfx::color565(130, 60, 220);
      pal_rimLight = lgfx::color565(90, 30, 200);
      break;
    default:  // 預設（沿用舊設定）
      pal_liquid = m_gridFluid;
      pal_foam = m_gridFoam;
      pal_rimTrans = lgfx::color565(0, 120, 255);
      pal_rimLight = lgfx::color565(0, 10, 230);
      break;
  }
  paletteDirty = true;
}

void FluidRenderer::setTheme(ShakeDir d) {
  applyPaletteForDir(d);
}

void FluidRenderer::setBasePalette(uint16_t liquid,
                                   uint16_t foam,
                                   uint16_t rimTrans,
                                   uint16_t rimLight) {
  pal_liquid = liquid;
  pal_foam = foam;
  pal_rimTrans = rimTrans;
  pal_rimLight = rimLight;
  paletteDirty = true;
}

// ========== 搖晃偵測：每筆加速度（g）丟進來 ==========
void FluidRenderer::onAccelSample(float ax_g, float ay_g, float az_g) {
#if SHAKE_USE_HIGHPASS
  // 抓重力 (低通) 並做高通
  lp_ax = (1.0f - SHAKE_LP_ALPHA) * lp_ax + SHAKE_LP_ALPHA * ax_g;
  lp_ay = (1.0f - SHAKE_LP_ALPHA) * lp_ay + SHAKE_LP_ALPHA * ay_g;
  lp_az = (1.0f - SHAKE_LP_ALPHA) * lp_az + SHAKE_LP_ALPHA * az_g;
  float hx = ax_g - lp_ax;
  float hy = ay_g - lp_ay;
  float hz = az_g - lp_az;
#else
  float hx = ax_g, hy = ay_g, hz = az_g;
#endif

#if SHAKE_INVERT_X
  hx = -hx;
#endif
#if SHAKE_INVERT_Y
  hy = -hy;
#endif

  // 三軸絕對值
  float vx[3] = {fabsf(hx), fabsf(hy), fabsf(hz)};
  // 找到最大 (primary) 與次大 (second) 值及其軸
  uint8_t primary_axis = 0;
  if (vx[1] > vx[primary_axis])
    primary_axis = 1;
  if (vx[2] > vx[primary_axis])
    primary_axis = 2;

  float primary = vx[primary_axis];
  // 求次大（從剩下兩個取 max）
  float second = (primary_axis == 0)   ? fmaxf(vx[1], vx[2])
                 : (primary_axis == 1) ? fmaxf(vx[0], vx[2])
                                       : fmaxf(vx[0], vx[1]);

  // 條件：主軸強度達標 + 主軸相對其他軸夠「主」
  bool strong = (primary >= SHAKE_THR_G) &&
                (second < 1e-6f || (primary / second >= SHAKE_DOMINANCE_RATIO));

  if (strong) {
    ShakeDir now = SHAKE_NONE;
    if (primary_axis == 0)
      now = (hx > 0) ? SHAKE_RIGHT : SHAKE_LEFT;  // X
    else if (primary_axis == 1)
      now = (hy > 0) ? SHAKE_UP : SHAKE_DOWN;  // Y
    else {  // Z 軸：映射到 UP/DOWN（你也可以自定成別的兩組主題色）
#if SHAKE_USE_Z
      now = (hz > 0) ? SHAKE_UP : SHAKE_DOWN;
#else
      now = SHAKE_NONE;
#endif
    }

    if (now == lastDir)
      ++hitCount;
    else {
      lastDir = now;
      hitCount = 1;
    }
  } else {
    hitCount = 0;
  }

  uint32_t nowMs = millis();
  if (hitCount >= (uint16_t)SHAKE_SAMPLES_HIT &&
      (uint32_t)(nowMs - lastFireMs) >= (uint32_t)SHAKE_COOLDOWN_MS) {
    applyPaletteForDir(lastDir);
    lastFireMs = nowMs;
    hitCount = 0;
  }
}

// ========== 取得顏色（優先用動態調色盤） ==========
uint16_t FluidRenderer::getFluidColor(RenderFluidType type) const {
  // 首次使用時初始化預設主題
  if (pal_liquid == 0 && pal_foam == 0 && pal_rimTrans == 0 &&
      pal_rimLight == 0) {
    const_cast<FluidRenderer*>(this)->applyPaletteForDir(SHAKE_NONE);
  }
  switch (type) {
    case RENDER_FLUID_LIQUID:
      return pal_liquid;
    case RENDER_FLUID_FOAM:
      return pal_foam;
    case RENDER_FLUID_RIM_TRANSPARENT:
      return pal_rimTrans ? pal_rimTrans : lgfx::color565(0, 120, 255);
    case RENDER_FLUID_RIM_LIGHT:
      return pal_rimLight ? pal_rimLight : lgfx::color565(0, 10, 230);
    default:
      return lgfx::color565(0, 0, 0);
  }
}

// ========== 幾何：該格是否在容器外（Solid） ==========
bool FluidRenderer::isSimSolid(int renderGx, int renderGy) const {
  const float cell = 1.0f / m_renderGridSize;
  const float cx = (renderGx + 0.5f) * cell - 0.5f;
  const float cy = (renderGy + 0.5f) * cell - 0.5f;
  constexpr float rad = 0.5f - ParticleSimulation::CELL;
  return (cx * cx + cy * cy) > (rad * rad);
}

// ========== 更新每格流體狀態（含 closing 平滑） ==========
void FluidRenderer::updateFluidCells() {
  const int GS = m_renderGridSize;
  const int GC = GS * GS;
  const float CELL = 1.0f / GS;

  memcpy(m_prevFluid, m_currFluid, GC * sizeof(RenderFluidType));

  static uint16_t cnt[MAX_GRID_CELLS];
  static float acc[MAX_GRID_CELLS];
  memset(cnt, 0, GC * sizeof(uint16_t));
  memset(acc, 0, GC * sizeof(float));

  const float r = PARTICLE_RADIUS;
  const float r2 = r * r;

  const Particle* P = m_sim->data();
  const int N = m_sim->particleCount();

  for (int p = 0; p < N; ++p) {
    const float px = P[p].x, py = P[p].y;
    const float sp = hypotf(P[p].vx, P[p].vy);

    int gx0 = constrain(int((px - r) * GS), 0, GS - 1);
    int gy0 = constrain(int((py - r) * GS), 0, GS - 1);
    int gx1 = constrain(int((px + r) * GS), 0, GS - 1);
    int gy1 = constrain(int((py + r) * GS), 0, GS - 1);

    for (int gx = gx0; gx <= gx1; ++gx)
      for (int gy = gy0; gy <= gy1; ++gy) {
        float cx = (gx + 0.5f) * CELL;
        float cy = (gy + 0.5f) * CELL;
        float dx = cx - px, dy = cy - py;
        if (dx * dx + dy * dy > r2)
          continue;

        int id = idx(gx, gy);
        ++cnt[id];
        acc[id] += sp;
      }
  }

  // 基本分類
  for (int id = 0; id < GC; ++id) {
    const float n = cnt[id];
    const float v = n ? acc[id] / n : 0.f;
    if (n >= RENDER_PARTICLE_THRESHOLD)
      m_currFluid[id] = (v > RENDER_FOAM_SPEED_THRESHOLD) ? RENDER_FLUID_FOAM
                                                          : RENDER_FLUID_LIQUID;
    else if (n >= RENDER_RIM_PARTICLE_THRESHOLD)
      m_currFluid[id] = RENDER_FLUID_RIM_TRANSPARENT;
    else
      m_currFluid[id] = RENDER_FLUID_EMPTY;
  }

  // 簡單包邊（近鄰轉換）
  memcpy(m_convTmp, m_currFluid, GC * sizeof(RenderFluidType));
  auto neighborFilled = [&](int id) -> bool {
    return (m_currFluid[id] == RENDER_FLUID_RIM_TRANSPARENT) ||
           (m_currFluid[id] == RENDER_FLUID_LIQUID) ||
           (m_currFluid[id] == RENDER_FLUID_FOAM);
  };
  for (int gx = 0; gx < GS; ++gx)
    for (int gy = 0; gy < GS; ++gy) {
      int id = idx(gx, gy);
      if (m_currFluid[id] != RENDER_FLUID_EMPTY)
        continue;

      int touch = 0;
      for (int dx = -1; dx <= 1 && touch < 4; ++dx)
        for (int dy = -1; dy <= 1 && touch < 4; ++dy) {
          if (!dx && !dy)
            continue;
          if (dx && dy)
            continue;  // 4-neigh
          int nx = gx + dx, ny = gy + dy;
          if (nx < 0 || nx >= GS || ny < 0 || ny >= GS)
            continue;
          if (neighborFilled(idx(nx, ny)))
            ++touch;
        }
      if (touch >= 4)
        m_convTmp[id] = RENDER_FLUID_LIQUID;
      else if (touch >= 2)
        m_convTmp[id] = RENDER_FLUID_RIM_TRANSPARENT;
      else if (touch >= 1)
        m_convTmp[id] = RENDER_FLUID_RIM_LIGHT;
    }
  memcpy(m_currFluid, m_convTmp, GC * sizeof(RenderFluidType));

  // Closing：dilation -> erosion（以曼哈頓半徑）
  const int R = RENDER_EDGE_SMOOTH_RADIUS;
  static uint8_t mask[MAX_GRID_CELLS];
  static uint8_t dilate[MAX_GRID_CELLS];
  static uint8_t closeB[MAX_GRID_CELLS];

  for (int i = 0; i < GC; ++i)
    mask[i] = (m_currFluid[i] == RENDER_FLUID_LIQUID ||
               m_currFluid[i] == RENDER_FLUID_FOAM ||
               m_currFluid[i] == RENDER_FLUID_RIM_TRANSPARENT)
                  ? 1
                  : 0;

  memset(dilate, 0, GC);
  for (int gx = 0; gx < GS; ++gx)
    for (int gy = 0; gy < GS; ++gy) {
      if (!mask[idx(gx, gy)])
        continue;
      for (int dx = -R; dx <= R; ++dx)
        for (int dy = -R; dy <= R; ++dy) {
          if (abs(dx) + abs(dy) > R)
            continue;
          int nx = gx + dx, ny = gy + dy;
          if (nx < 0 || nx >= GS || ny < 0 || ny >= GS)
            continue;
          dilate[idx(nx, ny)] = 1;
        }
    }

  memset(closeB, 0, GC);
  for (int gx = 0; gx < GS; ++gx)
    for (int gy = 0; gy < GS; ++gy) {
      bool all = true;
      for (int dx = -R; dx <= R && all; ++dx)
        for (int dy = -R; dy <= R && all; ++dy) {
          if (abs(dx) + abs(dy) > R)
            continue;
          int nx = gx + dx, ny = gy + dy;
          if (nx < 0 || nx >= GS || ny < 0 || ny >= GS || !dilate[idx(nx, ny)])
            all = false;
        }
      closeB[idx(gx, gy)] = all ? 1 : 0;
    }

  for (int i = 0; i < GC; ++i)
    if (closeB[i] && !mask[i] && m_currFluid[i] == RENDER_FLUID_EMPTY)
      m_currFluid[i] = RENDER_FLUID_RIM_LIGHT;

  // 紀錄變動索引
  m_changedCnt = 0;
  for (int i = 0; i < GC; ++i)
    if (m_currFluid[i] != m_prevFluid[i])
      m_changedIdx[m_changedCnt++] = i;
}

// ========== 渲染主控 ==========
void FluidRenderer::render(Mode mode) {
  m_disp->startWrite();

  // 若剛換色且是 PARTIAL 模式，保險起見先整畫一次
  bool forcedFull = false;
  if (paletteDirty && mode == PARTIAL_GRID) {
    updateFluidCells();
    renderGrid();
    paletteDirty = false;
    forcedFull = true;
  }

  if (!forcedFull) {
    switch (mode) {
      case BALLS:
        renderBalls();
        break;
      case GRID:
        updateFluidCells();
        renderGrid();
        paletteDirty = false;
        break;
      case PARTIAL_GRID:
        updateFluidCells();
        renderPartialGrid();
        paletteDirty = false;
        break;
      case PARTIAL_BALLS:  // 這裡直接當作 BALLS 全畫
        renderBalls();
        paletteDirty = false;
        break;
    }
  }

  m_disp->endWrite();
}

// ========== Balls 模式 ==========
void FluidRenderer::renderBalls() {
  // m_disp->fillScreen(lgfx::color565(0, 0, 0));
#if SHOW_SCREEN_FRAME
  m_disp->drawRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, TFT_WHITE);
#endif

  const Particle* P = m_sim->data();
  const int N = m_sim->particleCount();
  const int radius = (int)(m_renderCellSize * PARTICLE_RADIUS);

  for (int i = 0; i < N; ++i) {
    int sx = (int)(P[i].x * SCREEN_WIDTH);
    int sy = (int)(P[i].y * SCREEN_HEIGHT);

    float speed = sqrtf(P[i].vx * P[i].vx + P[i].vy * P[i].vy);
    float t = constrain(speed / 8.0f, 0.0f, 1.0f);
    uint16_t c = lerp565(m_ballBase, TFT_WHITE, t);

    m_disp->fillCircle(sx, sy, radius, c);
    m_disp->drawCircle(sx, sy, radius, TFT_NAVY);
  }
}

// ========== Grid (全畫) ==========
void FluidRenderer::renderGrid() {
  // m_disp->fillScreen(lgfx::color565(0, 0, 0));

  for (int gx = 0; gx < m_renderGridSize; ++gx) {
    for (int gy = 0; gy < m_renderGridSize; ++gy) {
      int px = gx * m_renderCellSize;
      int py = gy * m_renderCellSize;

      uint16_t color;
      if (isSimSolid(gx, gy)) {
        // color = m_gridSolid;
        continue;
      } else {
        RenderFluidType ft = m_currFluid[idx(gx, gy)];
        color = getFluidColor(ft);
      }

      m_disp->fillRect(px, py, m_renderCellSize, m_renderCellSize, color);
#if SHOW_CELL_OUTLINE
      m_disp->drawRect(px, py, m_renderCellSize, m_renderCellSize,
                       lgfx::color565(10, 10, 20));
#endif
    }
  }
}

// ========== Partial Grid（只畫變動格） ==========
void FluidRenderer::renderPartialGrid() {
  for (int n = 0; n < m_changedCnt; ++n) {
    int id = m_changedIdx[n];
    int gx = id / m_renderGridSize;
    int gy = id % m_renderGridSize;

    if (isSimSolid(gx, gy))
      continue;

    int px = gx * m_renderCellSize;
    int py = gy * m_renderCellSize;

    uint16_t color = getFluidColor(m_currFluid[id]);

    m_disp->fillRect(px, py, m_renderCellSize, m_renderCellSize, color);
#if SHOW_CELL_OUTLINE
    m_disp->drawRect(px, py, m_renderCellSize, m_renderCellSize,
                     lgfx::color565(10, 10, 20));
#endif
  }
}
