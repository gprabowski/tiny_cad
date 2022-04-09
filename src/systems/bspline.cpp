#include <systems.h>

bool systems::regenerate_bspline(
    relationship &r, adaptive &a,
    ecs::ComponentStorage<transformation> transformations,
    ecs::ComponentStorage<relationship> relationships,
    std::vector<glm::vec4> &out_vertices,
    std::vector<unsigned int> &out_indices,
    std::vector<glm::vec4> &out_vertices_polygon,
    std::vector<unsigned int> &out_indices_polygon) {

  out_vertices_polygon.clear();
  out_indices_polygon.clear();
  out_vertices.clear();
  out_indices.clear();

  for (std::size_t i = 0; i < r.children.size() - 3; ++i) {
    const auto first_child = r.children[i];
    const auto P10 = transformations[first_child].translation;

    const auto second_child = r.children[i + 1];
    const auto P20 = transformations[second_child].translation;

    const auto third_child = r.children[i + 2];
    const auto P30 = transformations[third_child].translation;

    const auto fourth_child = r.children[i + 3];
    const auto P40 = transformations[fourth_child].translation;

    for (int s = 0; s < 100; ++s) {
      const float u = 2.f + 0.01 * s;

      const float a41 = (u - 2.0f) / 3.0f;
      const float a31 = (u - 1.0f) / 3.0f;
      const float a21 = (u) / 3.0f;

      const float b41 = 1.0f - a41;
      const float b31 = 1.0f - a31;
      const float b21 = 1.0f - a21;

      const auto P41 = a41 * P40 + b41 * P30;
      const auto P31 = a31 * P30 + b31 * P20;
      const auto P21 = a21 * P20 + b21 * P10;

      const float a42 = (u - 2.0f) / 2.0f;
      const float a32 = (u - 1.0f) / 2.0f;

      const float b42 = 1.0f - a42;
      const float b32 = 1.0f - a32;

      const auto P42 = a42 * P41 + b42 * P31;
      const auto P32 = a32 * P31 + b32 * P21;

      const float a43 = (u - 2.0f) / 1.0f;
      const float b43 = 1.0f - a43;

      const auto P43 = a43 * P42 + b43 * P32;

      out_vertices.push_back(glm::vec4(P43, 1.0f));
    }
  }

  for (auto &c : r.children) {
    auto &t = transformations[c].translation;
    out_vertices_polygon.push_back(glm::vec4(t, 1.0f));
  }

  for (std::size_t i = 0; i < out_vertices.size(); ++i) {
    out_indices.push_back(i);
  }

  for (std::size_t i = 0; i < out_vertices_polygon.size(); ++i) {
    out_indices_polygon.push_back(i);
  }

  return true;
}
