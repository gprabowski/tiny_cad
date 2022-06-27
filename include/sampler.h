#pragma once

#include <functional>
#include <glm/glm.hpp>

#include <ecs.h>

struct sampler {
  std::function<glm::vec3(float u, float v)> sample;
  bool u_wrapped{false};
  bool v_wrapped{false};
};

sampler get_sampler(ecs::EntityType idx);
