#pragma once

#include <glm/glm.hpp>

#include <ecs.h>

struct bezier_surface_params {
  float height{10.0f}, width{10.0f};
  unsigned int u{1u}, v{1u};
  glm::vec3 root{0.f, 0.f, 0.f};
  bool cyllinder{false};
  ecs::EntityType bezier_polygon{1u << 31};
};
