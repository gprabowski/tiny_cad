#pragma once

#include <glm/glm.hpp>

struct bezier_surface_params {
  float height{1.0f}, width{1.0f};
  unsigned int u{2}, v{2};
  glm::vec3 root{0.f, 0.f, 0.f};
  bool cyllinder{false};
};
