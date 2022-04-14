#pragma once

#define GLM_FORCE_RADIANS
#include <frame_state.h>
#include <glm/glm.hpp>

struct score_helper {
  glm::vec4 pos;
  float score;
};

score_helper get_score(glm::vec3 &f);

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
