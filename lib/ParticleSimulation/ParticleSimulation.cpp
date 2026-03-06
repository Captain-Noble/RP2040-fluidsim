#include "ParticleSimulation.hpp"
#include <math.h>
#include <string.h>

// ──────────────────────────────────────── 工具

// ──────────────────────────────────────── 初始化
void ParticleSimulation::begin(QMI8658C* imu) {
  m_imu = imu;
  m_numParticles = PC_MAX;
  seedParticles();
  initGrid();
}

void ParticleSimulation::seedParticles() {
  for (int i = 0; i < m_numParticles; ++i) {
    // 0.2 ~ 0.8 区域内随机
    m_particles[i].x = random(20, 80) / 100.0f;
    m_particles[i].y = random(20, 80) / 100.0f;
    m_particles[i].vx = random(-50, 50) / 100.0f * CELL;  // 速度尺度≈单元
    m_particles[i].vy = random(-50, 50) / 100.0f * CELL;
    m_particles[i].r = 0.2f;
    m_particles[i].g = 0.4f;
    m_particles[i].b = 1.0f;
  }
}

void ParticleSimulation::initGrid() {
  for (int i = 0; i < GC; ++i) {
    int gx = i / GS, gy = i % GS;
    float cx = (gx + 0.5f) * CELL - 0.5f,
          cy = (gy + 0.5f) * CELL - 0.5f;  // 以(0,0)~(1,1)中心
    float rad = 0.5f - CELL;               // 圆容器半径
    m_cellType[i] = (cx * cx + cy * cy <= rad * rad) ? FLUID_CELL : SOLID_CELL;
    m_s[i] = 1.0f;
  }
}

// ──────────────────────────────────────── 主循环
void ParticleSimulation::simulate(float dt) {
  /* ───── 计时用变量 ─────────────────── */
  static uint32_t accIMU = 0;
  static uint32_t accIntg = 0;
  static uint32_t accPush = 0;
  static uint32_t accTvG = 0;  // transfer → grid
  static uint32_t accSolve = 0;
  static uint32_t accTvP = 0;  // transfer → particle
  static uint32_t accStat = 0;
  static uint32_t frames = 0;
  static uint32_t tLastPrint = millis();

  /* ───── 阶段 1：IMU ─────────────────── */
  uint32_t t0 = micros();
  updateIMU();
  uint32_t t1 = micros();

  /* ───── 阶段 2：积分 & 碰撞 ──────────── */
  integrateParticles(dt);
  uint32_t t2 = micros();

  /* ───── 阶段 3：粒子推开  ─────────────── */
  pushParticlesApart(SEPARATE_ITERS_P);
  // pushParticlesApartSpeed(SEPARATE_ITERS_P, dt);
  uint32_t t3 = micros();

  /* ───── 阶段 4：粒子 → 网格 (PIC) ─────── */
  transferVelocities(true, 0.0f);
  uint32_t t4 = micros();

  /* ───── 阶段 5：压力求解 ──────────────── */
  solveIncompressibility(SOLVER_ITERS_P, dt);
  uint32_t t5 = micros();

  /* ───── 阶段 6：网格 → 粒子 (FLIP/PIC) ─ */
  transferVelocities(false, FLIP_RATIO);
  uint32_t t6 = micros();

  /* ───── 阶段 7：统计/卷积 ─────────────── */
  updateFluidCells();
  uint32_t t7 = micros();

  /* ───── 累加 ─────────────────────────── */
  accIMU += t1 - t0;
  accIntg += t2 - t1;
  accPush += t3 - t2;
  accTvG += t4 - t3;
  accSolve += t5 - t4;
  accTvP += t6 - t5;
  accStat += t7 - t6;
  ++frames;

  /* ───── 每秒打印一次 ─────────────────── */
  if (millis() - tLastPrint >= 1000) {
    Serial.printf(
        "[%3u fps]  IMU:%4lu  Intg:%4lu  Push:%4lu  ToG:%4lu  Solve:%4lu  "
        "ToP:%4lu  Stat:%4lu (µs per frame)\r\n",
        frames, accIMU / frames, accIntg / frames, accPush / frames,
        accTvG / frames, accSolve / frames, accTvP / frames, accStat / frames);
    /* 清零 */
    accIMU = accIntg = accPush = accTvG = accSolve = accTvP = accStat = 0;
    frames = 0;
    tLastPrint = millis();
  }
}

// ──────────────────────────────────────── IMU
void ParticleSimulation::updateIMU() {
  if (!m_imu)
    return;
  float ax, ay, az;
  if (m_imu->readAccelerometer(&ax, &ay, &az)) {
    m_ax = ay * 10.f * GRAVITY_MODIFIER;  // 缩放到归一化空间
    m_ay = -ax * 10.f * GRAVITY_MODIFIER;
  }
}

// ──────────────────────────────────────── 积分+碰撞
void ParticleSimulation::integrateParticles(float dt) {
  constexpr float CX = 0.5f, CY = 0.5f, R = 0.5f - CELL - PARTICLE_RADIUS;
  for (int i = 0; i < m_numParticles; ++i) {
    Particle& p = m_particles[i];
    p.vx += m_ax * dt;
    p.vy += m_ay * dt;
    p.x += p.vx * dt;
    p.y += p.vy * dt;

    // 边界盒
    p.x = clampF(p.x, PARTICLE_RADIUS, 1.0f - PARTICLE_RADIUS);
    p.y = clampF(p.y, PARTICLE_RADIUS, 1.0f - PARTICLE_RADIUS);

    // 圆容器碰撞
    float dx = p.x - CX, dy = p.y - CY;
    float d2 = dx * dx + dy * dy;
    if (d2 > R * R) {
      // ---------- 推回圆内 ----------
      float d = sqrtf(d2);
      float inv = 1.0f / d;
      float nx = dx * inv;  // 法向单位向量
      float ny = dy * inv;

      float s = R - d;  // 穿透深度
      p.x += nx * s;    // 直接平移回边界
      p.y += ny * s;

      // ---------- 速度分解 ----------
      float vn = p.vx * nx + p.vy * ny;  // 法向分量
      float vx_n = vn * nx;              // 法向速度向量
      float vy_n = vn * ny;
      float vx_t = p.vx - vx_n;  // 切向速度向量
      float vy_t = p.vy - vy_n;

      vx_n = -REST_N * vx_n;
      vy_n = -REST_N * vy_n;
      vx_t = (1.0f - FRIC_T) * vx_t;
      vy_t = (1.0f - FRIC_T) * vy_t;

      p.vx = vx_n + vx_t;
      p.vy = vy_n + vy_t;
    }
  }
}

// ──────────────────────────────────────── Push-Apart
void ParticleSimulation::pushParticlesApart(int iters) {
  const float min2 =  // PUSH_DIST_MODIFIER * PUSH_DIST_MODIFIER *
      (2 * PARTICLE_RADIUS) * (2 * PARTICLE_RADIUS);

  memset(m_numPartCell, 0, sizeof(m_numPartCell));
  for (int i = 0; i < m_numParticles; ++i) {
    int xi = clampF(m_particles[i].x * P_INV_SP, 0, PNX - 1);
    int yi = clampF(m_particles[i].y * P_INV_SP, 0, PNY - 1);
    ++m_numPartCell[xi * PNY + yi];
  }
  int pref = 0;
  for (int i = 0; i < PNC; ++i) {
    int n = m_numPartCell[i];
    m_firstPart[i] = pref;
    pref += n;
  }
  m_firstPart[PNC] = pref;
  for (int i = 0; i < m_numParticles; ++i) {
    int xi = clampF(m_particles[i].x * P_INV_SP, 0, PNX - 1);
    int yi = clampF(m_particles[i].y * P_INV_SP, 0, PNY - 1);
    m_cellPartIds[--m_firstPart[xi * PNY + yi]] = i;
  }

  for (int it = 0; it < iters; ++it) {
    for (int i = 0; i < m_numParticles; ++i) {
      Particle& a = m_particles[i];
      int cx = a.x * P_INV_SP, cy = a.y * P_INV_SP;
      for (int xi = max(cx - 1, 0); xi <= min(cx + 1, PNX - 1); ++xi)
        for (int yi = max(cy - 1, 0); yi <= min(cy + 1, PNY - 1); ++yi) {
          int cell = xi * PNY + yi;
          for (int k = m_firstPart[cell]; k < m_firstPart[cell + 1]; ++k) {
            int j = m_cellPartIds[k];
            if (j <= i)
              continue;
            Particle& b = m_particles[j];
            float dx = b.x - a.x, dy = b.y - a.y, d2 = dx * dx + dy * dy;
            if (d2 > min2 || d2 == 0)
              continue;
            float d = sqrtf(d2),
                  s =  // PUSH_FORCE_MODIFIER *
                  0.5f * ((2 * PARTICLE_RADIUS) - d) / d;
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

// ──────────────────────────────────────── 速度搬运
void ParticleSimulation::transferVelocities(bool toGrid, float flipRatio) {
  const float hInv = float(GS);  // 1/H
  if (toGrid) {
    memcpy(m_prevU, m_u, sizeof(m_u));
    memcpy(m_prevV, m_v, sizeof(m_v));
    memset(m_u, 0, sizeof(m_u));
    memset(m_v, 0, sizeof(m_v));
    memset(m_du, 0, sizeof(m_du));
    memset(m_dv, 0, sizeof(m_dv));
  }

  for (int comp = 0; comp < 2; ++comp) {
    float dx = comp ? 0.5f * CELL : 0.0f;
    float dy = comp ? 0.0f : 0.5f * CELL;
    float *f = comp ? m_v : m_u, *fp = comp ? m_prevV : m_prevU,
          *dw = comp ? m_dv : m_du;

    for (int p = 0; p < m_numParticles; ++p) {
      Particle& pr = m_particles[p];
      float fx = (pr.x - dx) * hInv;
      float fy = (pr.y - dy) * hInv;

      int x0 = clampIdx(static_cast<int>(floorf(fx)), 0, GS - 1);
      int y0 = clampIdx(static_cast<int>(floorf(fy)), 0, GS - 1);
      float tx = fx - x0, ty = fy - y0, sx = 1 - tx, sy = 1 - ty;
      int x1 = clampIdx(x0 + 1, 0, GS - 1);
      int y1 = clampIdx(y0 + 1, 0, GS - 1);
      float w0 = sx * sy, w1 = tx * sy, w2 = tx * ty, w3 = sx * ty;
      int n0 = safeIdx(x0, y0);
      int n1 = safeIdx(x1, y0);
      int n2 = safeIdx(x1, y1);
      int n3 = safeIdx(x0, y1);

      if (toGrid) {
        float pv = comp ? pr.vy : pr.vx;
        f[n0] += pv * w0;
        dw[n0] += w0;
        f[n1] += pv * w1;
        dw[n1] += w1;
        f[n2] += pv * w2;
        dw[n2] += w2;
        f[n3] += pv * w3;
        dw[n3] += w3;
      } else {
        float pic = w0 * f[n0] + w1 * f[n1] + w2 * f[n2] + w3 * f[n3];
        float corr = w0 * (f[n0] - fp[n0]) + w1 * (f[n1] - fp[n1]) +
                     w2 * (f[n2] - fp[n2]) + w3 * (f[n3] - fp[n3]);
        float flip = (comp ? pr.vy : pr.vx) + corr;
        float val = (1.f - flipRatio) * pic + flipRatio * flip;
        if (comp)
          pr.vy = val;
        else
          pr.vx = val;
      }
    }
    if (toGrid)
      for (int i = 0; i < GC; ++i)
        if (dw[i] > 0)
          f[i] /= dw[i];
  }
}

// ──────────────────────────────────────── 压力求解
void ParticleSimulation::solveIncompressibility(int iters, float dt) {
  const float cp = FLUID_DENSITY * CELL / dt;
  for (int k = 0; k < iters; ++k) {
    for (int gx = 1; gx < GS - 1; ++gx)
      for (int gy = 1; gy < GS - 1; ++gy) {
        int c = idx(gx, gy);
        if (m_cellType[c] != FLUID_CELL)
          continue;
        int l = idx(gx - 1, gy), r = idx(gx + 1, gy), b = idx(gx, gy - 1),
            t = idx(gx, gy + 1);
        float div = m_u[r] - m_u[c] + m_v[t] - m_v[c];
        float p = -div / 4.f * 1.9f;
        m_pressure[c] += cp * p;
        m_u[c] -= p;
        m_u[r] += p;
        m_v[c] -= p;
        m_v[t] += p;
      }
  }
}

// ──────────────────────────────────────── 状态统计

void ParticleSimulation::updateFluidCells() {
  /* 0️⃣ 备份上一帧状态 */
  // memcpy(m_prevFluid, m_currFluid, sizeof(m_currFluid));

  /* 1️⃣ 统计粒子覆盖半径：cnt[]、acc[] ---------------------------------- */
  static uint16_t cnt[GC];
  static float acc[GC];
  memset(cnt, 0, sizeof(cnt));
  memset(acc, 0, sizeof(acc));

  const float r = PARTICLE_RADIUS;  // 归一化空间半径
  const float r2 = r * r;           // 半径平方
  const float cell = CELL;          // 1 / GS

  for (int p = 0; p < m_numParticles; ++p) {
    const float px = m_particles[p].x;
    const float py = m_particles[p].y;
    const float speed = hypotf(m_particles[p].vx, m_particles[p].vy);

    /* —— 找出能被此粒子波及的格子 AABB —— */
    int gx0 = clampIdx(static_cast<int>((px - r) * GS), 0, GS - 1);
    int gy0 = clampIdx(static_cast<int>((py - r) * GS), 0, GS - 1);
    int gx1 = clampIdx(static_cast<int>((px + r) * GS), 0, GS - 1);
    int gy1 = clampIdx(static_cast<int>((py + r) * GS), 0, GS - 1);

    for (int gx = gx0; gx <= gx1; ++gx)
      for (int gy = gy0; gy <= gy1; ++gy) {
        /* —— 精确判距：以格子中心为准 —— */
        float cx = (gx + 0.5f) * cell;
        float cy = (gy + 0.5f) * cell;
        float dx = cx - px;
        float dy = cy - py;
        if (dx * dx + dy * dy > r2)
          continue;                // 超出半径
        int id = safeIdx(gx, gy);  // 统一使用 safeIdx
        ++cnt[id];
        acc[id] += speed;
      }
  }
}
