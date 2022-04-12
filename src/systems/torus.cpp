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

glm::vec4 sample_torus(const torus_params &tp, const float u, const float v) {
  const auto sin_u = sinf(u);
  const auto cos_u = cosf(u);

  const auto sin_v = sinf(v);
  const auto cos_v = cosf(v);

  return {tp.radii[1] * cos_u + tp.radii[0] * cos_u * cos_v,
          tp.radii[1] * sin_u + tp.radii[0] * sin_u * cos_v,
          tp.radii[0] * sin_v, 1.0f};
}

void generate_torus_lines(const parametric &p,
                          const std::vector<glm::vec4> &points,
                          std::vector<unsigned int> &indices) {
  for (int i = 0u; i < p.samples[0]; ++i) {
    for (int j = 0u; j < p.samples[1]; ++j) {
      // add quad as two triangles
      const auto imod = (i + 1) % p.samples[0];
      const auto jmod = (j + 1) % p.samples[1];

      const auto i1 = i * p.samples[1] + j;
      const auto i2 = imod * p.samples[1] + j;
      const auto i3 = imod * p.samples[1] + jmod;
      const auto i4 = i * p.samples[1] + jmod;

      indices.push_back(i1);
      indices.push_back(i2);

      indices.push_back(i2);
      indices.push_back(i3);

      indices.push_back(i3);
      indices.push_back(i4);

      indices.push_back(i4);
      indices.push_back(i1);
    }
  }
}

} // namespace systems
