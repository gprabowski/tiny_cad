#pragma once

#include "torus.h"
#include <component_manager.h>
#include <ecs.h>
#include <gl_object.h>
#include <parametric.h>
#include <vector>

namespace systems {

glm::vec4 sample_torus(const torus_params &tp, const float u, const float v);

template <typename ParamsType>
inline glm::vec4 sample(const ParamsType &p, const float u, const float v) {
  if constexpr (std::is_same_v<ParamsType, torus_params>) {
    return sample_torus(p, u, v);
  }
  return glm::vec4();
}

template <typename ParamsType>
bool generate_points(const ParamsType &s, const parametric &p,
                     std::vector<glm::vec4> &out_vertices) {

  const auto u_diff = p.u_max - p.u_min;
  const auto v_diff = p.v_max - p.v_min;

  const auto u_inv_div = 1.0f / static_cast<float>(p.samples[0]);
  const auto v_inv_div = 1.0f / static_cast<float>(p.samples[1]);

  for (unsigned int i = 0u; i < p.samples[0]; ++i) {
    for (unsigned int j = 0u; j < p.samples[1]; ++j) {
      out_vertices.emplace_back(
          sample<ParamsType>(s, p.u_min + u_diff * i * u_inv_div,
                             p.v_min + v_diff * j * v_inv_div));
    }
  }
  return true;
}

// gl object
void generate_lines(const parametric &p, const std::vector<glm::vec4> &points,
                    std::vector<unsigned int> &indices);

void reset_gl_objects(gl_object &g);
void render_points(const gl_object &g);
void set_model_uniform(const transformation &t);

} // namespace systems
