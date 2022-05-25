#pragma once

#include <glm/glm.hpp>

#include <ecs.h>

struct bspline_surface_params {
  float height{10.0f}, width{10.0f};
  unsigned int u{2}, v{2};
  glm::vec3 root{0.f, 0.f, 0.f};
  bool cyllinder{false};
  ecs::EntityType deboor_polygon{1u << 31};
};
