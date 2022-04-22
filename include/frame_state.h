#pragma once

#include <set>
#include <vector>

#include <glad/glad.h>

#include <glm/glm.hpp>

#include <imgui.h>

#include <ecs.h>

struct frame_state {
  std::vector<ecs::EntityType> regeneration;

  static glm::mat4 view;
  static glm::mat4 proj;
  static int window_w;
  static int window_h;

  static uint64_t freq;
  static double last_cpu_frame;

  static std::vector<ecs::EntityType> changed;
  static std::vector<ecs::EntityType> deleted;
  static std::set<ecs::EntityType> changed_parents;

  static ImVec2 content_area;
  static ImVec2 content_pos;
};
