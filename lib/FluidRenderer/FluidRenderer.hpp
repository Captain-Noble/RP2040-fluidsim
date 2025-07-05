#pragma once
#include <LovyanGFX.h>
#include "ParticleSimulation.hpp"

class FluidRenderer {
 public:
  enum Mode { BALLS, GRID, PARTIAL_GRID };  // ← 新枚举

  FluidRenderer(lgfx::LGFX_Device* disp, const ParticleSimulation* sim)
      : m_disp(disp), m_sim(sim) {}

  void render(Mode mode);

  // 原有接口
  void renderBalls();
  void renderGrid();
  void renderPartialGrid();  // ← 新接口

  // 配色
  void setBallBaseColor(uint16_t c) { m_ballBase = c; }
  void setGridSolidColor(uint16_t c) { m_gridSolid = c; }
  void setGridFluidColor(uint16_t c) { m_gridFluid = c; }

 private:
  lgfx::LGFX_Device* m_disp;
  const ParticleSimulation* m_sim;

  uint16_t m_ballBase = TFT_CYAN;
  uint16_t m_gridSolid = TFT_DARKGREY;
  uint16_t m_gridFluid = TFT_BLUE;
  uint16_t m_gridFoam = TFT_WHITE;

  inline uint16_t lerp565(uint16_t c1, uint16_t c2, float t) const;
};
