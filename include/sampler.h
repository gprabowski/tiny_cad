#pragma once

#include <functional>
#include <glm/glm.hpp>

#include <ecs.h>

struct sampler {
  std::function<glm::vec3(float u, float v)> sample;

  std::function<glm::vec3(float u, float v)> der_u;
  std::function<glm::vec3(float u, float v)> der_v;

  glm::vec3 normal(float u, float v) {
    return glm::cross(der_u(u, v), der_v(u, v));
  }

  bool u_wrapped{false};
  bool v_wrapped{false};
};

sampler get_sampler(ecs::EntityType idx);
