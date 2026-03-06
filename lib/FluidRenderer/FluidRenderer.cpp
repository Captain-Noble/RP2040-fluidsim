#include "FluidRenderer.hpp"

// 配置参数（可调整以适应不同的渲染效果）

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

// 获取流体类型对应的颜色
uint16_t FluidRenderer::getFluidColor(RenderFluidType type) const {
  switch (type) {
    case RENDER_FLUID_LIQUID:
      return m_disp->color565(0, 240, 255);
    case RENDER_FLUID_FOAM:
      return m_disp->color565(200, 200, 230);
    case RENDER_FLUID_RIM_TRANSPARENT:
      return m_disp->color565(0, 120, 255);
    case RENDER_FLUID_RIM_LIGHT:
      return m_disp->color565(0, 0, 200);
    default:  // RENDER_FLUID_EMPTY
      return m_disp->color565(0, 0, 0);
  }
}

// 检查模拟网格中的固体单元（坐标转换）
// ─────────────────────────────────────────────
// 根据圆形容器方程判断渲染网格单元是否为 Solid
// 与模拟端 initGrid() 使用同一半径：rad = 0.5 - CELL
// ─────────────────────────────────────────────
bool FluidRenderer::isSimSolid(int renderGx, int renderGy) const {
  // 渲染网格单元中心在 [0,1] 归一化坐标系中的位置
  const float cell = 1.0f / m_renderGridSize;        // 渲染网格单元尺寸
  const float cx = (renderGx + 0.5f) * cell - 0.5f;  // 相对圆心 x
  const float cy = (renderGy + 0.5f) * cell - 0.5f;  // 相对圆心 y

  // 与模拟端一致的圆容器半径（使用模拟 CELL，保持几何一致性）
  constexpr float rad = 0.5f - ParticleSimulation::CELL;

  // 圆外 ⇒ Solid
  return (cx * cx + cy * cy) > (rad * rad);
}

/******************************************************************
 * FluidRenderer::updateFluidCells() -- 卷积-Closing 平滑液面边缘
 *   ① 统计半径覆盖 → 基础分类
 *   ② 形态学 closing(膨胀→腐蚀)  → 填平凹洞 / 削细尖
 *   ③ 新生成的填充格设为 RENDER_FLUID_RIM_LIGHT
 ******************************************************************/

void FluidRenderer::updateFluidCells() {
  const int GS = m_renderGridSize;
  const int GC = GS * GS;
  const float CELL = 1.0f / GS;

  /* 0️⃣ 备份上一帧状态 */
  memcpy(m_prevFluid, m_currFluid, GC * sizeof(RenderFluidType));

  /* 1️⃣ 统计粒子覆盖半径：cnt[] / acc[] ------------------------- */
  static uint16_t cnt[MAX_GRID_CELLS];
  static float acc[MAX_GRID_CELLS];
  memset(cnt, 0, GC * sizeof(uint16_t));
  memset(acc, 0, GC * sizeof(float));

  const float r = PARTICLE_RADIUS;  // 归一化半径
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

  /* 2️⃣ 基础分类 (Liquid / RimTransparent / Empty / Foam) -------- */
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

  /* 3️⃣ 原有「邻域包边」卷积：EMPTY → Rim* ------------------------ */
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
            continue;  // 自身
          if (dx && dy)
            continue;  // 只看 4 邻域
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

  /* --- 把包边结果写回作为 closing 的初始基准 -------------------- */
  memcpy(m_currFluid, m_convTmp, GC * sizeof(RenderFluidType));

  /* 4️⃣ Closing(膨胀→腐蚀) 平滑液面 ------------------------------ */
  const int R = RENDER_EDGE_SMOOTH_RADIUS;  // 卷积半径

  static uint8_t mask[MAX_GRID_CELLS];
  static uint8_t dilate[MAX_GRID_CELLS];
  static uint8_t close[MAX_GRID_CELLS];

  /* 4-A 生成二值掩码（液面/泡沫/透明边缘 = 1） */
  for (int i = 0; i < GC; ++i)
    mask[i] = (m_currFluid[i] == RENDER_FLUID_LIQUID ||
               m_currFluid[i] == RENDER_FLUID_FOAM ||
               m_currFluid[i] == RENDER_FLUID_RIM_TRANSPARENT)
                  ? 1
                  : 0;

  /* 4-B 膨胀 dilation */
  memset(dilate, 0, GC);
  for (int gx = 0; gx < GS; ++gx)
    for (int gy = 0; gy < GS; ++gy) {
      if (!mask[idx(gx, gy)])
        continue;
      for (int dx = -R; dx <= R; ++dx)
        for (int dy = -R; dy <= R; ++dy) {
          if (abs(dx) + abs(dy) > R)
            continue;  // 曼哈顿邻域
          int nx = gx + dx, ny = gy + dy;
          if (nx < 0 || nx >= GS || ny < 0 || ny >= GS)
            continue;
          dilate[idx(nx, ny)] = 1;
        }
    }

  /* 4-C 腐蚀 erosion */
  memset(close, 0, GC);
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
      close[idx(gx, gy)] = all ? 1 : 0;
    }

  /* 4-D 新填补的 EMPTY → RIM_LIGHT */
  for (int i = 0; i < GC; ++i)
    if (close[i] && !mask[i] && m_currFluid[i] == RENDER_FLUID_EMPTY)
      m_currFluid[i] = RENDER_FLUID_RIM_LIGHT;

  /* 5️⃣ 生成变化列表 --------------------------------------------- */
  m_changedCnt = 0;
  for (int i = 0; i < GC; ++i)
    if (m_currFluid[i] != m_prevFluid[i])
      m_changedIdx[m_changedCnt++] = i;
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
      // 先更新状态，再渲染变化部分
      updateFluidCells();
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
  const int radius = (int)(m_renderCellSize * PARTICLE_RADIUS);

  for (int i = 0; i < N; ++i) {
    int sx = (int)(P[i].x * SCREEN_WIDTH);
    int sy = (int)(P[i].y * SCREEN_HEIGHT);

    // 速度映射到颜色
    float speed = sqrtf(P[i].vx * P[i].vx + P[i].vy * P[i].vy);
    float t = constrain(speed / 8.0f, 0.0f, 1.0f);
    uint16_t c = lerp565(m_ballBase, TFT_WHITE, t);

    m_disp->fillCircle(sx, sy, radius, c);
    m_disp->drawCircle(sx, sy, radius, TFT_NAVY);  // 细圈
  }
}

void FluidRenderer::renderGrid() {
  // 1. 先更新状态
  updateFluidCells();

  // 2. 先清屏
  m_disp->fillScreen(m_disp->color565(0, 0, 0));

  // 3. 逐格绘制
  for (int gx = 0; gx < m_renderGridSize; ++gx) {
    for (int gy = 0; gy < m_renderGridSize; ++gy) {
      int px = gx * m_renderCellSize;
      int py = gy * m_renderCellSize;

      uint16_t color;

      if (isSimSolid(gx, gy)) {
        color = m_gridSolid;  // 墙
      } else {
        // 读取当前帧流体状态
        RenderFluidType ft = m_currFluid[idx(gx, gy)];
        color = getFluidColor(ft);
      }

      m_disp->fillRect(px, py, m_renderCellSize, m_renderCellSize, color);

      // （可选）描边
      if (DRAW_RECT)
        m_disp->drawRect(px, py, m_renderCellSize, m_renderCellSize,
                         m_disp->color565(10, 10, 20));
    }
  }
}

void FluidRenderer::renderPartialGrid() {
  // 注意：updateFluidCells() 已在 render() 中调用

  // 统计并打印
  // Serial.printf("Changed cells this frame: %d\n", m_changedCnt);

  for (int n = 0; n < m_changedCnt; ++n) {
    int idx = m_changedIdx[n];
    int gx = idx / m_renderGridSize;
    int gy = idx % m_renderGridSize;

    if (isSimSolid(gx, gy))
      continue;  // 固体不用重画

    // 像素坐标
    int px = gx * m_renderCellSize;
    int py = gy * m_renderCellSize;

    // 颜色
    uint16_t color = getFluidColor(m_currFluid[idx]);

    // 绘制
    m_disp->fillRect(px, py, m_renderCellSize, m_renderCellSize, color);
    if (DRAW_RECT && m_currFluid[idx] != RENDER_FLUID_EMPTY)
      m_disp->drawRect(px, py, m_renderCellSize, m_renderCellSize,
                       m_disp->color565(0, 0, 200));
  }
}