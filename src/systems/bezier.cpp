#include <algorithm>
#include <frame_state.h>
#include <initializer_list>
#include <log.h>
#include <systems.h>
#include <utility>
#include <vector>

namespace systems {

struct score_helper {
  glm::vec4 pos;
  float score;
};

score_helper get_score(glm::vec3 &f) {
  const auto pv = frame_state::proj * frame_state::view;
  auto first = pv * glm::vec4(f, 1.0f);
  first = first / first.w;
  first = glm::clamp(first, -1.0f, 1.0f);
  return {first, 0.0f};
}

template <typename First, typename... T>
score_helper get_score(First &f, T... rest) {
  const auto pv = frame_state::proj * frame_state::view;
  auto first = pv * glm::vec4(f, 1.0f);
  first = first / first.w;
  first = glm::clamp(first, -1.0f, 1.0f);
  const auto others = get_score(rest...);
  const auto len = glm::length(first - others.pos) + others.score;
  return {first, len};
}

template <typename... T> float get_pixel_score(T &&...args) {
  const auto res = get_score(std::forward<T>(args)...);
  return res.score * 0.5 * (frame_state::window_w + frame_state::window_h);
}

bool regenerate_bezier(const relationship &r, adaptive &a,
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
  for (int remaining = r.children.size(); remaining > 0; remaining -= 3) {
    if (remaining > 3) {
      const auto first_child = r.children[remaining - 1];
      const auto b_a = transformations[first_child].translation;

      const auto second_child = r.children[remaining - 2];
      const auto b_b = transformations[second_child].translation;

      const auto third_child = r.children[remaining - 3];
      const auto b_c = transformations[third_child].translation;

      const auto fourth_child = r.children[remaining - 4];
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
      out_vertices_polygon.push_back(glm::vec4(b_a, 1.0f));
      out_vertices_polygon.push_back(glm::vec4(b_b, 1.0f));
      out_vertices_polygon.push_back(glm::vec4(b_c, 1.0f));
      out_vertices_polygon.push_back(glm::vec4(b_d, 1.0f));
    } else if (remaining == 3) {

      const auto first_child = r.children[remaining - 1];
      const auto b_e = transformations[first_child].translation;

      const auto second_child = r.children[remaining - 2];
      const auto b_f = transformations[second_child].translation;

      const auto third_child = r.children[remaining - 3];
      const auto b_g = transformations[third_child].translation;

      auto tmp = get_pixel_score(b_e, b_f, b_g) / 20;
      auto current_score = std::clamp(tmp, 3.f, 100.f);
      float div = 1.0f / current_score;
      for (float t = 0.0; t <= 1.0f; t = t + div) {
        const auto b_h = (1.f - t) * b_e + t * b_f;
        const auto b_i = (1.f - t) * b_f + t * b_g;
        out_vertices.push_back(glm::vec4(((1.f - t) * b_h + t * b_i), 1.0f));
      }

      out_vertices_polygon.push_back(glm::vec4(b_e, 1.0f));
      out_vertices_polygon.push_back(glm::vec4(b_f, 1.0f));
      out_vertices_polygon.push_back(glm::vec4(b_g, 1.0f));
    } else if (remaining == 2) {
      const auto first_child = r.children[remaining - 1];
      const auto b_h = transformations[first_child].translation;

      const auto second_child = r.children[remaining - 2];
      const auto b_i = transformations[second_child].translation;
      for (float t = 0.0; t <= 1.0f; t = t + 1.0f) {
        out_vertices.push_back(glm::vec4(((1.f - t) * b_h + t * b_i), 1.0f));
      }
      out_vertices_polygon.push_back(glm::vec4(b_h, 1.0f));
      out_vertices_polygon.push_back(glm::vec4(b_i, 1.0f));
    } else if (remaining == 1) {
      const auto first_child = r.children[remaining - 1];
      const auto first_t = transformations[first_child];
      out_vertices.push_back({first_t.translation, 1.0f});
      out_vertices_polygon.push_back(glm::vec4(first_t.translation, 1.0f));
    }
  }

  for (std::size_t i = 0; i < out_vertices.size(); ++i) {
    out_indices.push_back(i);
  }

  for (std::size_t i = 0; i < out_vertices_polygon.size(); ++i) {
    out_indices_polygon.push_back(i);
  }

  return true;
}
} // namespace systems
