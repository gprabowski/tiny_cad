#include <algorithm>
#include <frame_state.h>
#include <initializer_list>
#include <log.h>
#include <systems.h>
#include <utility>
#include <vector>

namespace systems {

void regenerate_bezier(ecs::EntityType idx) {
  auto &reg = ecs::registry::get_registry();

  relationship &r = reg.get_component<relationship>(idx);
  gl_object &g = reg.get_component<gl_object>(idx);
  std::vector<glm::vec4> &out_vertices = g.points;
  std::vector<unsigned int> &out_indices = g.indices;
  auto &bez = reg.get_component<bezierc>(idx);
  gl_object &bgl = reg.get_component<gl_object>(bez.bezier_polygon);
  std::vector<glm::vec4> &bezier_polygon_vertices = bgl.points;
  std::vector<unsigned int> &bezier_polygon_indices = bgl.indices;

  auto &transformations = reg.get_map<transformation>();
  bezier_polygon_vertices.clear();
  bezier_polygon_indices.clear();
  out_vertices.clear();
  out_indices.clear();
  // for (int remaining = r.children.size(); remaining > 0; remaining -= 3) {
  for (std::size_t j = 0; j < r.children.size(); j += 3) {
    const auto remaining = r.children.size() - j;
    if (remaining > 3) {
      const auto first_child = r.children[j];
      const auto b_a = transformations[first_child].translation;

      const auto second_child = r.children[j + 1];
      const auto b_b = transformations[second_child].translation;

      const auto third_child = r.children[j + 2];
      const auto b_c = transformations[third_child].translation;

      const auto fourth_child = r.children[j + 3];
      const auto b_d = transformations[fourth_child].translation;

      // out vertices
      out_vertices.push_back(glm::vec4(b_a, 1.0f));
      out_vertices.push_back(glm::vec4(b_b, 1.0f));
      out_vertices.push_back(glm::vec4(b_c, 1.0f));
      out_vertices.push_back(glm::vec4(b_d, 1.0f));

      bezier_polygon_vertices.push_back(glm::vec4(b_a, 1.0f));
      bezier_polygon_vertices.push_back(glm::vec4(b_b, 1.0f));
      bezier_polygon_vertices.push_back(glm::vec4(b_c, 1.0f));
      bezier_polygon_vertices.push_back(glm::vec4(b_d, 1.0f));
    } else if (remaining == 3) {

      const auto first_child = r.children[j];
      const auto b_e = transformations[first_child].translation;

      const auto second_child = r.children[j + 1];
      const auto b_f = transformations[second_child].translation;

      const auto third_child = r.children[j + 2];
      const auto b_g = transformations[third_child].translation;

      const auto d0 = b_e;
      const auto d1 = 1.f / 3.f * b_e + 2.f / 3.f * b_f;
      const auto d2 = 2.f / 3.f * b_f + 1.f / 3.f * b_g;
      const auto d3 = b_g;

      // elevating the degree to B3 from B2
      out_vertices.emplace_back(glm::vec4(d0, 1.f));
      out_vertices.emplace_back(glm::vec4(d1, 1.f));
      out_vertices.emplace_back(glm::vec4(d2, 1.f));
      out_vertices.emplace_back(glm::vec4(d3, 1.f));

      bezier_polygon_vertices.push_back(glm::vec4(b_e, 1.0f));
      bezier_polygon_vertices.push_back(glm::vec4(b_f, 1.0f));
      bezier_polygon_vertices.push_back(glm::vec4(b_g, 1.0f));
    } else if (remaining == 2) {
      const auto first_child = r.children[j];
      const auto b_h = transformations[first_child].translation;

      const auto second_child = r.children[j + 1];
      const auto b_i = transformations[second_child].translation;

      const auto d0 = b_h;
      const auto d1 = 2.f / 3.f * b_h + 1.f / 3.f * b_i;
      const auto d2 = 1.f / 3.f * b_h + 2.f / 3.f * b_i;
      const auto d3 = b_i;

      // elevating the degree to B3 from B1
      out_vertices.emplace_back(glm::vec4(d0, 1.f));
      out_vertices.emplace_back(glm::vec4(d1, 1.f));
      out_vertices.emplace_back(glm::vec4(d2, 1.f));
      out_vertices.emplace_back(glm::vec4(d3, 1.f));

      bezier_polygon_vertices.push_back(glm::vec4(b_h, 1.0f));
      bezier_polygon_vertices.push_back(glm::vec4(b_i, 1.0f));
    }
  }

  for (std::size_t i = 0; i < out_vertices.size(); ++i) {
    out_indices.push_back(i);
  }

  for (std::size_t i = 0; i < bezier_polygon_vertices.size(); ++i) {
    bezier_polygon_indices.push_back(i);
  }

  systems::reset_gl_objects(g);
  systems::reset_gl_objects(bgl);
}
} // namespace systems
