#pragma once

#include <functional>
#include <glm/glm.hpp>

#include <ecs.h>

struct sampler {
  std::function<glm::vec3(float u, float v)> sample;

  bool u_wrapped{false};
  bool v_wrapped{false};

  float u_divisor{1.f};
  float v_divisor{1.f};

  float normal_translation{0.f};

  glm::vec3 der_u(float u, float v) const {
    const auto h = 1e-3f;
    const auto v_o = sample(u, v);
    const auto v_u = sample(u + h, v);
    return (v_u - v_o) * (1.f / h);
  };

  glm::vec3 der_u_opt(float u, float v, const glm::vec3 &v_o) const {
    const auto h = 1e-3f;
    const auto v_u = sample(u + h, v);
    return (v_u - v_o) * (1.f / h);
  };

  glm::vec3 der_v(float u, float v) const {
    const auto h = 1e-3f;
    const auto v_o = sample(u, v);
    const auto v_v = sample(u, v + h);
    return (v_v - v_o) * (1.f / h);
  };

  glm::vec3 der_v_opt(float u, float v, const glm::vec3 &v_o) const {
    const auto h = 1e-3f;
    const auto v_v = sample(u, v + h);
    return (v_v - v_o) * (1.f / h);
  };

  glm::vec3 normal(float u, float v) const {
    return glm::normalize(
        -glm::cross(glm::normalize(der_u(u, v)), glm::normalize(der_v(u, v))));
  }
};

sampler get_sampler(ecs::EntityType idx);
