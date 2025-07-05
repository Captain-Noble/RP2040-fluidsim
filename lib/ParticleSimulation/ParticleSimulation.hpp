#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <stdint.h>
#include <cstring>  // for memcpy / memset
#include "qmi8658c.hpp"

// ═════════════ 全局常量（根据你的新宏）═════════════
#define LOGICAL_GRID_SIZE 20  // 正方形网格 X = Y
#define SCREEN_WIDTH 240
#define SCREEN_HEIGHT 240
#define PIXEL_PER_CELL (SCREEN_WIDTH / LOGICAL_GRID_SIZE)
#define FLUID_PARTICLE_THRESHOLD 1  // ≥ 多少粒子算"液面"
#define FOAM_SPEED_THRESHOLD \
  0.3f  // 单位：逻辑速度；超过视为泡沫（调整为单位网格）

// —— 流体离散参数 ————————————————————————————————
#define NUM_PARTICLES 100      // 粒子总数
#define PARTICLE_RADIUS 0.02f  // 物理半径 (相对于单位网格)
#define FLUID_DENSITY 1.0f     // ρ，用于压力计算
#define SOLVER_ITERS_P 4       // 压强迭代
#define SEPARATE_ITERS_P 2     // 粒子互斥推开迭代
#define FLIP_RATIO 0.45f       // FLIP ↔ PIC 混合

// —— 网格 cell 类型 ——————————————————————
enum CellType : uint8_t { FLUID_CELL, AIR_CELL, SOLID_CELL };
// new
enum FluidType : uint8_t { FLUID_EMPTY, FLUID_LIQUID, FLUID_FOAM };

// ═══════════════════════════════════════════════════
struct Particle {
  float x, y;     // 逻辑坐标 (0-1范围)
  float vx, vy;   // 速度
  float r, g, b;  // 颜色 (可视化)
};

// ═══════════════════════════════════════════════════
class ParticleSimulation {
 public:
  // 传入已初始化好的 IMU 指针（可为 nullptr → 不采 IMU）
  void begin(QMI8658C* imu);

  // 每帧调用：内部完成 IMU 更新 + FLIP 求解
  void simulate(float dt);

  //---------------- 渲染只读接口 ----------------
  const Particle* data() const { return m_particles; }
  int particleCount() const { return m_numParticles; }
  bool isSolid(int gx, int gy) const {
    return m_cellType[gx * GS + gy] == SOLID_CELL;
  }

  static constexpr int GS = LOGICAL_GRID_SIZE;  // grid side
  static constexpr int GC = GS * GS;            // grid cells
  static constexpr int PC_MAX = NUM_PARTICLES;
  static constexpr float H = 1.0f / GS;  // 网格单位长度 = 1/GS (整个网格是1x1)

  FluidType m_currFluid[GC];
  FluidType m_prevFluid[GC];

  inline const int* changedIndices() const { return m_changedIdx; }
  inline int changedCount() const { return m_changedCnt; }

 private:
  //---------------- 网格字段 ----------------
  float m_u[GC];  // 水平速度 (边-中心差分形式简化)
  float m_v[GC];  // 垂直速度
  float m_prevU[GC], m_prevV[GC];
  float m_du[GC], m_dv[GC];  // 累加权重
  float m_pressure[GC];
  float m_s[GC];  // 固体-边界系数
  CellType m_cellType[GC];

  int m_changedIdx[GC];  // 存放索引（gx*GS + gy）
  int m_changedCnt{0};   // 本帧元素个数

  //---------------- 粒子字段 ----------------
  Particle m_particles[PC_MAX];
  int m_numParticles;

  // ——   空间哈希（粒子 PushApart 用）——
  static constexpr float P_INV_SP = 1.0f / (2.2f * PARTICLE_RADIUS);
  static constexpr int PNX =
      int(1.0f / (2.2f * PARTICLE_RADIUS)) + 2;  // 整个网格是1单位
  static constexpr int PNY = PNX;
  static constexpr int PNC = PNX * PNY;
  int m_numPartCell[PNC];
  int m_firstPart[PNC + 1];
  int m_cellPartIds[PC_MAX];

  //---------------- 设备指针 ----------------
  QMI8658C* m_imu{nullptr};
  float m_ax{0.f}, m_ay{0.f};

  //---------------- 内部算法 ----------------
  // 1. 基础
  void seedParticles();
  void initGrid();
  void updateIMU();

  // 2. 粒子阶段
  void integrateParticles(float dt);
  void pushParticlesApart(int iters);

  // 3. 速度搬运
  void transferVelocities(bool toGrid, float flipRatio);

  // 4. 压力-不可压
  void solveIncompressibility(int iters, float dt);
  void updateFluidCells();

  // 5. 工具
  inline int idx(int x, int y) const { return x * GS + y; }
  static inline float clampF(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
  }
};