#pragma once

#include <functional>
#include <glm/glm.hpp>

#include <ecs.h>

struct sampler {
  std::function<glm::vec3(float u, float v)> sample;

  std::function<glm::vec3(float u, float v)> der_u;
  std::function<glm::vec3(float u, float v)> der_v;
  std::function<glm::vec3(float u, float v, const glm::vec3 &v_o)> der_u_opt;
  std::function<glm::vec3(float u, float v, const glm::vec3 &v_o)> der_v_opt;

  glm::vec3 normal(float u, float v) {
    return glm::cross(der_u(u, v), der_v(u, v));
  }

  bool u_wrapped{false};
  bool v_wrapped{false};
};

sampler get_sampler(ecs::EntityType idx);
