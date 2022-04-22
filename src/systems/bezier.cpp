#include <adaptive_score.h>
#include <algorithm>
#include <frame_state.h>
#include <initializer_list>
#include <log.h>
#include <systems.h>
#include <utility>
#include <vector>

namespace systems {

bool regenerate_bezier(ecs::EntityType idx) {
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

      auto tmp = get_pixel_score(b_a, b_b, b_c, b_d) / 100;
      auto current_score = std::clamp(tmp, 10.f, 100.f);

      float div = 1.0f / current_score;
      for (float t = 0.0; t < 1.0f; t = t + div) {
        const auto b_e = (1.f - t) * b_a + t * b_b;
        const auto b_f = (1.f - t) * b_b + t * b_c;
        const auto b_g = (1.f - t) * b_c + t * b_d;
        const auto b_h = (1.f - t) * b_e + t * b_f;
        const auto b_i = (1.f - t) * b_f + t * b_g;
        out_vertices.push_back(glm::vec4(((1.f - t) * b_h + t * b_i), 1.0f));
      }
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

      auto tmp = get_pixel_score(b_e, b_f, b_g) / 20;
      auto current_score = std::clamp(tmp, 3.f, 100.f);
      float div = 1.0f / current_score;
      for (float t = 0.0; t < 1.0f; t = t + div) {
        const auto b_h = (1.f - t) * b_e + t * b_f;
        const auto b_i = (1.f - t) * b_f + t * b_g;
        out_vertices.push_back(glm::vec4(((1.f - t) * b_h + t * b_i), 1.0f));
      }

      out_vertices.push_back(glm::vec4(b_g, 1.0f));

      bezier_polygon_vertices.push_back(glm::vec4(b_e, 1.0f));
      bezier_polygon_vertices.push_back(glm::vec4(b_f, 1.0f));
      bezier_polygon_vertices.push_back(glm::vec4(b_g, 1.0f));
    } else if (remaining == 2) {
      const auto first_child = r.children[j];
      const auto b_h = transformations[first_child].translation;

      const auto second_child = r.children[j + 1];
      const auto b_i = transformations[second_child].translation;
      for (float t = 0.0; t <= 1.0f; t = t + 1.0f) {
        out_vertices.push_back(glm::vec4(((1.f - t) * b_h + t * b_i), 1.0f));
      }
      bezier_polygon_vertices.push_back(glm::vec4(b_h, 1.0f));
      bezier_polygon_vertices.push_back(glm::vec4(b_i, 1.0f));
    } else if (remaining == 1) {
      const auto first_child = r.children[j];
      const auto first_t = transformations[first_child];
      out_vertices.push_back({first_t.translation, 1.0f});
      bezier_polygon_vertices.push_back(glm::vec4(first_t.translation, 1.0f));
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

  return true;
}
} // namespace systems
