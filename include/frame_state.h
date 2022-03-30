#pragma once

#include <vector>

#include <glm/glm.hpp>

#include <ecs.h>

struct frame_state {
  std::vector<ecs::EntityType> regeneration;

  static glm::mat4 view;
  static glm::mat4 proj;
  static int window_w;
  static int window_h;
};
