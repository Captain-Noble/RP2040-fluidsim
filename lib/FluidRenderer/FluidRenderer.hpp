#pragma once
#include <LovyanGFX.h>
#include <stdint.h>
#include <string.h>
#include "ParticleSimulation.hpp"

// === 渲染網格設定 ===
#define RENDER_GRID_SIZE 48
#define RENDER_PIXEL_PER_CELL (SCREEN_HEIGHT / RENDER_GRID_SIZE)

// ------------------------------------------------------------
// 新增：搖晃方向列舉（給主程式和渲染器共用）
// ------------------------------------------------------------
enum ShakeDir : uint8_t {
  SHAKE_NONE = 0,
  SHAKE_LEFT,
  SHAKE_RIGHT,
  SHAKE_UP,
  SHAKE_DOWN
};

// 渲染器使用的流体类型定义
enum RenderFluidType : uint8_t {
  RENDER_FLUID_EMPTY,
  RENDER_FLUID_LIQUID,
  RENDER_FLUID_FOAM,
  RENDER_FLUID_RIM_TRANSPARENT,
  RENDER_FLUID_RIM_LIGHT
};

class FluidRenderer {
 public:
  enum Mode { BALLS, GRID, PARTIAL_GRID, PARTIAL_BALLS };

  // 搖晃方向（用來切換主題）
  using ShakeDir = ::ShakeDir;

  FluidRenderer(lgfx::LGFX_Device* disp, const ParticleSimulation* sim)
      : m_disp(disp), m_sim(sim) {
    memset(m_prevFluid, 0, sizeof(m_prevFluid));
    memset(m_currFluid, 0, sizeof(m_currFluid));
  }

  // === 對外 API ===
  void render(Mode mode);

  // 原有接口
  void renderBalls();
  void renderGrid();
  void renderPartialGrid();

  // 新增：更新流體狀態（從粒子資料計算渲染網格）
  void updateFluidCells();

  // 配色（保留舊介面；作為預設主題色）
  void setBallBaseColor(uint16_t c) { m_ballBase = c; }
  void setGridSolidColor(uint16_t c) { m_gridSolid = c; }
  void setGridFluidColor(uint16_t c) { m_gridFluid = c; }

  // 新增：搖晃偵測（每次讀到 IMU 加速度值（單位 g）就呼叫）
  void onAccelSample(float ax_g, float ay_g, float az_g);

  // 新增：手動切主題 / 自訂主題調色盤
  void setTheme(ShakeDir d);
  void setBasePalette(uint16_t liquid,
                      uint16_t foam,
                      uint16_t rimTrans,
                      uint16_t rimLight);

  // 設定渲染網格大小（陣列容量以 RENDER_GRID_SIZE 為上限）
  void setRenderGridSize(int size) {
    m_renderGridSize = size;
    m_renderCellSize = SCREEN_WIDTH / size;
  }

 private:
  // === 輔助 ===
  inline uint16_t lerp565(uint16_t c1, uint16_t c2, float t) const;
  inline int idx(int x, int y) const { return x * m_renderGridSize + y; }
  uint16_t getFluidColor(RenderFluidType type) const;
  bool isSimSolid(int renderGx, int renderGy) const;

  // 新增：依搖晃方向套用主題
  void applyPaletteForDir(ShakeDir d);

 private:
  // === 物件與參數 ===
  lgfx::LGFX_Device* m_disp;
  const ParticleSimulation* m_sim;

  int m_renderGridSize = RENDER_GRID_SIZE;
  int m_renderCellSize = RENDER_PIXEL_PER_CELL;

  // 狀態追蹤（以最大網格容量配置）
  static constexpr int MAX_GRID_CELLS = RENDER_GRID_SIZE * RENDER_GRID_SIZE;
  RenderFluidType m_prevFluid[MAX_GRID_CELLS];
  RenderFluidType m_currFluid[MAX_GRID_CELLS];
  int m_changedIdx[MAX_GRID_CELLS];
  int m_changedCnt = 0;

  // 臨時緩衝
  RenderFluidType m_convTmp[MAX_GRID_CELLS];

  // === 舊版顏色（作為預設主題）===
  uint16_t m_ballBase = TFT_CYAN;
  uint16_t m_gridSolid = TFT_DARKGREY;
  uint16_t m_gridFluid = TFT_BLUE;  // 液體主色（預設）
  uint16_t m_gridFoam = TFT_WHITE;  // 泡沫顏色（預設）

  // === 新增：動態調色盤（搖晃改色用）===
  uint16_t pal_liquid = 0;
  uint16_t pal_foam = 0;
  uint16_t pal_rimTrans = 0;
  uint16_t pal_rimLight = 0;
  bool paletteDirty = true;  // 換色後要求全畫一次以避免 partial 漏畫

  // === 新增：高通 / 去抖 / 冷卻 狀態 ===
  float lp_ax = 0.f, lp_ay = 0.f, lp_az = 1.f;  // 低通累積（估重力）
  ShakeDir lastDir = SHAKE_NONE;
  uint16_t hitCount = 0;
  uint32_t lastFireMs = 0;
};
