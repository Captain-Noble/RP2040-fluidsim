#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <stdint.h>
#include <cstring>
#include "qmi8658c.hpp"

// ─── 宏与常量 ─────────────────────────────────────
#define LOGICAL_GRID_SIZE 16  // GS
#define SCREEN_WIDTH 240
#define SCREEN_HEIGHT 240
#define PIXEL_PER_CELL (SCREEN_WIDTH / LOGICAL_GRID_SIZE)

#define FOAM_SPEED_THRESHOLD 999.0f / LOGICAL_GRID_SIZE
#define NUM_PARTICLES 100
#define PARTICLE_RADIUS 0.5f / LOGICAL_GRID_SIZE  // 归一化单位；≈ 单元半径一半
#define FLUID_DENSITY 1.0f
#define SOLVER_ITERS_P 1
#define SEPARATE_ITERS_P 2
#define FLIP_RATIO 0.5f

#define GRAVITY_MODIFIER 1

#define REST_N 0.02f
#define FRIC_T 0.05f

// ─── 枚举 ─────────────────────────────────────────
enum CellType : uint8_t { FLUID_CELL, AIR_CELL, SOLID_CELL };
enum FluidType : uint8_t {
  FLUID_EMPTY,
  FLUID_LIQUID,
  FLUID_RIM_TRANSPARENT,
  FLUID_FOAM,
  FLUID_RIM_LIGHT
};

// ─── 结构 ─────────────────────────────────────────
struct Particle {
  float x, y;     // 位置  ∈ [0,1]
  float vx, vy;   // 速度
  float r, g, b;  // 调试颜色
};

// ─── 主类 ─────────────────────────────────────────
class ParticleSimulation {
 public:
  void begin(QMI8658C* imu);
  void simulate(float dt);

  // 渲染层接口
  const Particle* data() const { return m_particles; }
  int particleCount() const { return m_numParticles; }
  bool isSolid(int gx, int gy) const {
    return m_cellType[gx * GS + gy] == SOLID_CELL;
  }
  const int* changedIndices() const { return m_changedIdx; }
  int changedCount() const { return m_changedCnt; }

  // 静态别名
  static constexpr int GS = LOGICAL_GRID_SIZE;  // 网格边
  static constexpr int GC = GS * GS;            // 单元数
  static constexpr float CELL = 1.0f / GS;      // 单元物理尺寸
  static constexpr float H = CELL;              // 与旧代码兼容
  static constexpr int PC_MAX = NUM_PARTICLES;

  // 公开流体面板
  // FluidType m_currFluid[GC]{};
  // FluidType m_prevFluid[GC]{};
  // FluidType convTmp[GC];

 private:
  // ── 网格字段 ───────────────────────────────
  float m_u[GC]{}, m_v[GC]{}, m_prevU[GC]{}, m_prevV[GC]{};
  float m_du[GC]{}, m_dv[GC]{}, m_pressure[GC]{}, m_s[GC]{};
  CellType m_cellType[GC]{};

  // ── 粒子字段 ───────────────────────────────
  Particle m_particles[PC_MAX];
  int m_numParticles{0};

  // 空间哈希
  static constexpr float P_INV_SP = 1.0f / (2.2f * PARTICLE_RADIUS);  // 归一化
  static constexpr int PNX = int(1.0f * P_INV_SP) + 2;
  static constexpr int PNY = PNX;
  static constexpr int PNC = PNX * PNY;
  int m_numPartCell[PNC]{}, m_firstPart[PNC + 1]{}, m_cellPartIds[PC_MAX]{};

  // 变化列表
  int m_changedIdx[GC]{}, m_changedCnt{0};

  // 传感器
  QMI8658C* m_imu{nullptr};
  float m_ax{0.f}, m_ay{0.f};

  // ── 内部算法 ───────────────────────────────
  void seedParticles();
  void initGrid();
  void updateIMU();
  void integrateParticles(float dt);
  void pushParticlesApart(int iters);

  void transferVelocities(bool toGrid, float flipRatio);
  void solveIncompressibility(int iters, float dt);
  void updateFluidCells();
  // 工具
  inline int idx(int x, int y) const { return x * GS + y; }
  static inline float clampF(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
  }

  template <typename T>
  static inline T clampIdx(T v, T lo, T hi) {
    return (v < lo) ? lo : (v > hi ? hi : v);
  }

  // 替换旧的 idx 宏 / 函数：先夹取再转一维下标
  static inline int safeIdx(int x, int y) {
    x = clampIdx(x, 0, GS - 1);
    y = clampIdx(y, 0, GS - 1);
    return x * GS + y;  // ← 与原来 idx 相同的线性展开方式
  }
};
