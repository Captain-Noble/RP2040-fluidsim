#include "ParticleSimulation.hpp"

// ═══════════════════════════════════════════════════
//  初始
// ═══════════════════════════════════════════════════
// 计算粒子数量 + 平均速度 → 返回流体类型
static inline FluidType classifyCell(int count, float avgSpeed) {
  if (count < FLUID_PARTICLE_THRESHOLD)
    return FLUID_EMPTY;
  if (avgSpeed > FOAM_SPEED_THRESHOLD)
    return FLUID_FOAM;
  return FLUID_LIQUID;
}

void ParticleSimulation::begin(QMI8658C* imu) {
  m_imu = imu;
  m_numParticles = PC_MAX;
  seedParticles();
  initGrid();
}

void ParticleSimulation::seedParticles() {
  for (int i = 0; i < m_numParticles; ++i) {
    float gx = random(2, GS - 2);
    float gy = random(2, GS - 2);
    m_particles[i].x = gx;
    m_particles[i].y = gy;
    m_particles[i].vx = random(-50, 50) / 100.0f;
    m_particles[i].vy = random(-50, 50) / 100.0f;
    m_particles[i].r = 0.2f;
    m_particles[i].g = 0.4f;
    m_particles[i].b = 1.0f;
  }
}

void ParticleSimulation::initGrid() {
  for (int i = 0; i < GC; ++i) {
    m_u[i] = m_v[i] = m_prevU[i] = m_prevV[i] = 0.f;
    m_du[i] = m_dv[i] = 0.f;
    m_pressure[i] = 0.f;
    // 圆形容器 -> 内部流体，外部固体
    int gx = i / GS, gy = i % GS;
    float cx = gx - GS * 0.5f, cy = gy - GS * 0.5f;
    float rad = GS * 0.5f;
    m_cellType[i] = (cx * cx + cy * cy <= rad * rad) ? FLUID_CELL : SOLID_CELL;
    // m_cellType[i] = FLUID_CELL;
    m_s[i] = 1.0f;  // 简化，流体-固体交界权重 =1
  }
}

// ═══════════════════════════════════════════════════
//  每帧对外接口
// ═══════════════════════════════════════════════════
void ParticleSimulation::simulate(float dt) {
  updateIMU();
  integrateParticles(dt);
  pushParticlesApart(SEPARATE_ITERS_P);
  transferVelocities(true, 0.f);  // 粒子→网格
  solveIncompressibility(SOLVER_ITERS_P, dt);
  transferVelocities(false, FLIP_RATIO);  // 网格→粒子 (FLIP/PIC 混合)
  updateFluidCells();
}

void ParticleSimulation::updateFluidCells() {
  // 1) 把上一帧状态保存
  memcpy(m_prevFluid, m_currFluid, sizeof(m_currFluid));

  // 2) 清零计数&速度累加缓存
  static uint16_t partCount[GC];
  static float speedAcc[GC];
  memset(partCount, 0, sizeof(partCount));
  memset(speedAcc, 0, sizeof(speedAcc));

  // 3) 遍历粒子累加到对应网格
  for (int i = 0; i < m_numParticles; ++i) {
    int gx = (int)clampF(floorf(m_particles[i].x), 0, GS - 1);
    int gy = (int)clampF(floorf(m_particles[i].y), 0, GS - 1);
    int id = idx(gx, gy);

    float speed = sqrtf(m_particles[i].vx * m_particles[i].vx +
                        m_particles[i].vy * m_particles[i].vy);
    ++partCount[id];
    speedAcc[id] += speed;
  }

  m_changedCnt = 0;
  // 4) 逐格判定类型
  for (int c = 0; c < GC; ++c) {
    float avgSpeed = partCount[c] ? speedAcc[c] / partCount[c] : 0.f;
    m_currFluid[c] = classifyCell(partCount[c], avgSpeed);

    bool changed = (m_prevFluid[c] != m_currFluid[c]);
    if (changed)  // 只在变化时记录索引
      m_changedIdx[m_changedCnt++] = c;
  }
}

// ═══════════════════════════════════════════════════
//   IMU
// ═══════════════════════════════════════════════════
void ParticleSimulation::updateIMU() {
  if (!m_imu)
    return;
  float ax, ay, az;
  if (m_imu->readAccelerometer(&ax, &ay, &az)) {
    m_ax = ay * 10.f;
    m_ay = -ax * 10.f;
  }
}

// ═══════════════════════════════════════════════════
//   粒子积分
// ═══════════════════════════════════════════════════
void ParticleSimulation::integrateParticles(float dt) {
  constexpr float CX = GS * 0.5f - 0.5f;  // 圆心
  constexpr float CY = GS * 0.5f - 0.5f;
  constexpr float R = GS * 0.5f - 1.0f - PARTICLE_RADIUS;  // 实际可达半径

  for (int i = 0; i < m_numParticles; ++i) {
    Particle& p = m_particles[i];

    // ── 1. 外力积分 ─────────────────────
    p.vy += m_ay * dt;
    p.vx += m_ax * dt;
    p.x += p.vx * dt;
    p.y += p.vy * dt;

    // ── 2. 四边框夹紧（防数据溢出） ──────
    if (p.x < 0) {
      p.x = 0;
      p.vx = 0;
    }
    if (p.x > GS - 1) {
      p.x = GS - 1;
      p.vx = 0;
    }
    if (p.y < 0) {
      p.y = 0;
      p.vy = 0;
    }
    if (p.y > GS - 1) {
      p.y = GS - 1;
      p.vy = 0;
    }

    // ── 3. 检测 / 处理圆形 SOLID_CELL ──
    float dx = p.x - CX;
    float dy = p.y - CY;
    float dist2 = dx * dx + dy * dy;

    if (dist2 > R * R) {  // 落在固体区
      float dist = sqrtf(dist2);
      float s = R / dist;  // 推回系数
      p.x = CX + dx * s;   // 位置投回圆内
      p.y = CY + dy * s;

      // —— 速度反射（简单弹性） ——
      float vn = (p.vx * dx + p.vy * dy) / dist;  // 投影到法向
      p.vx -= 2.0f * vn * dx / dist;              // 反射 =  v - 2(v·n)n
      p.vy -= 2.0f * vn * dy / dist;
    }
  }
}

// ═══════════════════════════════════════════════════
//   Push-Apart  (离散格哈希)
// ═══════════════════════════════════════════════════
void ParticleSimulation::pushParticlesApart(int iters) {
  const float minDist2 = (2 * PARTICLE_RADIUS) * (2 * PARTICLE_RADIUS);

  // 统计
  memset(m_numPartCell, 0, sizeof(m_numPartCell));
  for (int i = 0; i < m_numParticles; ++i) {
    int xi = (int)clampF(m_particles[i].x * P_INV_SP, 0, PNX - 1);
    int yi = (int)clampF(m_particles[i].y * P_INV_SP, 0, PNY - 1);
    ++m_numPartCell[xi * PNY + yi];
  }
  // 前缀和
  int sum = 0;
  for (int i = 0; i < PNC; ++i) {
    int t = m_numPartCell[i];
    m_firstPart[i] = sum;
    sum += t;
  }
  m_firstPart[PNC] = sum;
  // 填充
  for (int i = 0; i < m_numParticles; ++i) {
    int xi = (int)clampF(m_particles[i].x * P_INV_SP, 0, PNX - 1);
    int yi = (int)clampF(m_particles[i].y * P_INV_SP, 0, PNY - 1);
    int cell = xi * PNY + yi;
    m_cellPartIds[--m_firstPart[cell]] = i;
  }

  // 迭代推开
  for (int it = 0; it < iters; ++it) {
    for (int i = 0; i < m_numParticles; ++i) {
      Particle& a = m_particles[i];
      int cxi = (int)(a.x * P_INV_SP);
      int cyi = (int)(a.y * P_INV_SP);
      for (int xi = max(cxi - 1, 0); xi <= min(cxi + 1, PNX - 1); ++xi)
        for (int yi = max(cyi - 1, 0); yi <= min(cyi + 1, PNY - 1); ++yi) {
          int cell = xi * PNY + yi;
          for (int k = m_firstPart[cell]; k < m_firstPart[cell + 1]; ++k) {
            int j = m_cellPartIds[k];
            if (j <= i)
              continue;
            Particle& b = m_particles[j];
            float dx = b.x - a.x, dy = b.y - a.y;
            float d2 = dx * dx + dy * dy;
            if (d2 > minDist2 || d2 == 0.f)
              continue;
            float d = sqrtf(d2);
            float s = 0.5f * (2 * PARTICLE_RADIUS - d) / d;
            dx *= s;
            dy *= s;
            a.x -= dx;
            a.y -= dy;
            b.x += dx;
            b.y += dy;
          }
        }
    }
  }
}

// ═══════════════════════════════════════════════════
//   粒子 <-> 网格 速度搬运 (简化版)
// ═══════════════════════════════════════════════════
void ParticleSimulation::transferVelocities(bool toGrid, float flipRatio) {
  const float h = H;
  const float hInv = 1.0f / h;
  // toGrid: 累加;  fromGrid: 插值回粒子
  if (toGrid) {
    memcpy(m_prevU, m_u, sizeof(m_u));
    memcpy(m_prevV, m_v, sizeof(m_v));
    memset(m_u, 0, sizeof(m_u));
    memset(m_v, 0, sizeof(m_v));
    memset(m_du, 0, sizeof(m_du));
    memset(m_dv, 0, sizeof(m_dv));
  }

  for (int comp = 0; comp < 2; ++comp) {
    float dx = comp == 0 ? 0.f : 0.5f;
    float dy = comp == 0 ? 0.5f : 0.f;
    float* f = comp == 0 ? m_u : m_v;
    float* prevF = comp == 0 ? m_prevU : m_prevV;
    float* dwei = comp == 0 ? m_du : m_dv;

    for (int p = 0; p < m_numParticles; ++p) {
      Particle& pr = m_particles[p];
      float x = clampF(pr.x, 1, GS - 2);
      float y = clampF(pr.y, 1, GS - 2);

      float fx = (x - dx) * hInv;
      float fy = (y - dy) * hInv;
      int x0 = (int)fx;
      float tx = fx - x0;
      int y0 = (int)fy;
      float ty = fy - y0;
      int x1 = x0 + 1 < GS - 1 ? x0 + 1 : GS - 2;
      int y1 = y0 + 1 < GS - 1 ? y0 + 1 : GS - 2;
      float sx = 1.f - tx, sy = 1.f - ty;

      float w0 = sx * sy, w1 = tx * sy, w2 = tx * ty, w3 = sx * ty;
      int nr0 = idx(x0, y0), nr1 = idx(x1, y0), nr2 = idx(x1, y1),
          nr3 = idx(x0, y1);

      if (toGrid) {
        float pv = comp == 0 ? pr.vx : pr.vy;
        f[nr0] += pv * w0;
        dwei[nr0] += w0;
        f[nr1] += pv * w1;
        dwei[nr1] += w1;
        f[nr2] += pv * w2;
        dwei[nr2] += w2;
        f[nr3] += pv * w3;
        dwei[nr3] += w3;
      } else {
        float pic = w0 * f[nr0] + w1 * f[nr1] + w2 * f[nr2] + w3 * f[nr3];
        float corr = w0 * (f[nr0] - prevF[nr0]) + w1 * (f[nr1] - prevF[nr1]) +
                     w2 * (f[nr2] - prevF[nr2]) + w3 * (f[nr3] - prevF[nr3]);
        float flip = (comp == 0 ? pr.vx : pr.vy) + corr;
        float blended = (1.f - flipRatio) * pic + flipRatio * flip;
        if (comp == 0)
          pr.vx = blended;
        else
          pr.vy = blended;
      }
    }
    if (toGrid) {
      for (int i = 0; i < GC; ++i)
        if (dwei[i] > 0)
          f[i] /= dwei[i];
    }
  }
}

// ═══════════════════════════════════════════════════
//   压力求解 (简版 Jacobi)
// ═══════════════════════════════════════════════════
void ParticleSimulation::solveIncompressibility(int iters, float dt) {
  const float cp = FLUID_DENSITY * H / dt;
  for (int i = 0; i < iters; ++i) {
    for (int gx = 1; gx < GS - 1; ++gx)
      for (int gy = 1; gy < GS - 1; ++gy) {
        int c = idx(gx, gy);
        if (m_cellType[c] != FLUID_CELL)
          continue;
        int l = idx(gx - 1, gy), r = idx(gx + 1, gy), b = idx(gx, gy - 1),
            t = idx(gx, gy + 1);
        float div = m_u[r] - m_u[c] + m_v[t] - m_v[c];
        float s = 4.f;  // 邻居系数固定 1
        float p = -div / s;
        p *= 1.9f;  // over-relax
        m_pressure[c] += cp * p;
        m_u[c] -= p;
        m_u[r] += p;
        m_v[c] -= p;
        m_v[t] += p;
      }
  }
}