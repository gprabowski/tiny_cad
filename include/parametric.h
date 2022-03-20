#pragma once

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>

#include <vector>

struct parametric {
  float u_min{0.0f};
  float u_max{0.0f};
  float v_min{0.0f};
  float v_max{0.0f};
  int samples[2]{0u, 0u};
};
