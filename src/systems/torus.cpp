#include <systems.h>

namespace systems {

bool generate_torus_points(const torus_params &s, const parametric &p,
                           std::vector<glm::vec4> &out_vertices) {

  const auto u_diff = p.u_max - p.u_min;
  const auto v_diff = p.v_max - p.v_min;

  const auto u_inv_div = 1.0f / static_cast<float>(p.samples[0]);
  const auto v_inv_div = 1.0f / static_cast<float>(p.samples[1]);

  for (int i = 0u; i < p.samples[0]; ++i) {
    for (int j = 0u; j < p.samples[1]; ++j) {
      out_vertices.emplace_back(sample_torus(s,
                                             p.u_min + u_diff * i * u_inv_div,
                                             p.v_min + v_diff * j * v_inv_div));
    }
  }
  return true;
}

} // namespace systems
