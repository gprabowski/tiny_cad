#include <adaptive_score.h>

score_helper get_score(glm::vec3 &f) {
  const auto pv = frame_state::proj * frame_state::view;
  auto first = pv * glm::vec4(f, 1.0f);
  first = first / first.w;
  first = glm::clamp(first, -1.0f, 1.0f);
  return {first, 0.0f};
}
