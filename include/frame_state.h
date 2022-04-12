#pragma once

#include <vector>

#include <glad/glad.h>

#include <glm/glm.hpp>

#include <ecs.h>

struct frame_state {
  std::vector<ecs::EntityType> regeneration;

  static glm::mat4 view;
  static glm::mat4 proj;
  static int window_w;
  static int window_h;

  static GLuint common_ubo;
  static GLuint common_idx;
  static int common_block_loc;

  static GLuint default_program;

  static uint64_t freq;
  static double last_cpu_frame;
};
