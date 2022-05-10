#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <frame_state.h>
#include <frame_update.h>
#include <input_state.h>
#include <registry.h>
#include <shader_manager.h>
#include <systems.h>

namespace update {

void setup_globals(const ImVec2 &s) {
  auto &state = input_state::get_input_state();

  int width, height;
  auto w = glfwGetCurrentContext();
  glfwGetFramebufferSize(w, &width, &height);

  frame_state::window_h = height;
  frame_state::window_w = width;
  const auto min = ImGui::GetWindowContentRegionMin();
  const auto max = ImGui::GetWindowContentRegionMax();
  frame_state::content_area = {max.x - min.x, max.y - min.y};
  frame_state::content_pos = {ImGui::GetWindowPos()};
  frame_state::content_pos = {frame_state::content_pos.x + min.x,
                              frame_state::content_pos.y + min.y};

  auto view = (glm::lookAt(state.cam_pos, state.cam_pos + state.cam_front,
                           state.cam_up));
  auto proj =
      (glm::perspective(glm::radians(state.fov_y),
                        static_cast<float>(frame_state::content_area.x) /
                            frame_state::content_area.y,
                        state.clip_near, state.clip_far));

  frame_state::view = view;
  frame_state::proj = proj;
}

void refresh_impl(glm::mat4 &col, glm::mat4 &view, glm::mat4 &proj) {
  static auto &sm = shader_manager::get_manager();
  glBindBufferBase(GL_UNIFORM_BUFFER, sm.common_ubo_block_loc, sm.common_ubo);
  float *common_ubo_ptr = (float *)glMapNamedBufferRange(
      sm.common_ubo, 0, 3 * 16 * sizeof(float),
      GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT);
  memcpy(&common_ubo_ptr[0], glm::value_ptr(proj), sizeof(float) * 16);
  memcpy(&common_ubo_ptr[16], glm::value_ptr(view), sizeof(float) * 16);
  memcpy(&common_ubo_ptr[32], glm::value_ptr(col), sizeof(float) * 16);
  glUnmapNamedBuffer(sm.common_ubo);
  glBindBuffer(GL_UNIFORM_BUFFER, sm.common_ubo);
}

void refresh_ubos() {
  refresh_impl(frame_state::col_mat, frame_state::view, frame_state::proj);
}

void refresh_ubos_left() {
  static auto &is = input_state::get_input_state();
  float top, bottom, left, right;

  float aspect_ratio = static_cast<float>(frame_state::content_area.x) /
                       frame_state::content_area.y;

  top = is.clip_near * tan(glm::radians(is.fov_y / 2));
  bottom = -top;
  float a = aspect_ratio * tanf(glm::radians(is.fov_y) / 2) *
            is.ster_sett.convergence;
  float b = a - is.ster_sett.eye_separation / 2;
  float c = a + is.ster_sett.eye_separation / 2;
  left = -b * is.clip_near / is.ster_sett.convergence;
  right = c * is.clip_near / is.ster_sett.convergence;
  auto lproj =
      glm::frustum(left, right, bottom, top, is.clip_near, is.clip_far);
  auto lview = glm::translate(glm::mat4(1.0f),
                              {is.ster_sett.eye_separation / 2, 0.0f, 0.0f}) *
               frame_state::view;
  refresh_impl(frame_state::col_mat_left, lview, lproj);
}

void refresh_ubos_right() {
  static auto &is = input_state::get_input_state();
  float aspect_ratio = static_cast<float>(frame_state::content_area.x) /
                       frame_state::content_area.y;
  float top, bottom, left, right;
  top = is.clip_near * tan(glm::radians(is.fov_y / 2));
  bottom = -top;
  float a =
      aspect_ratio * tan(glm::radians(is.fov_y / 2)) * is.ster_sett.convergence;
  float b = a - is.ster_sett.eye_separation / 2;
  float c = a + is.ster_sett.eye_separation / 2;
  left = -c * is.clip_near / is.ster_sett.convergence;
  right = b * is.clip_near / is.ster_sett.convergence;
  auto rproj =
      glm::frustum(left, right, bottom, top, is.clip_near, is.clip_far);

  auto rview = glm::translate(glm::mat4(1.0f),
                              {-is.ster_sett.eye_separation / 2, 0.0f, 0.0f}) *
               frame_state::view;

  refresh_impl(frame_state::col_mat_right, rview, rproj);
}

void per_frame_update() {
  systems::update_changed_relationships();
  systems::delete_entities();
}

} // namespace update
