#include <frame_render.h>

#include <frame_state.h>
#include <frame_update.h>
#include <framebuffer.h>
#include <gui.h>
#include <input_handlers.h>
#include <systems.h>

namespace hnd = handlers;
namespace sys = systems;

namespace render {
void begin_frame(uint64_t &b) {
  static glm::vec4 clear_color = {0.0f, 0.0f, 0.0f, 1.00f};
  b = glfwGetTimerValue();
  glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w,
               clear_color.z * clear_color.w, clear_color.w);
  glClearDepth(1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  frame_state::changed.clear();
  frame_state::deleted.clear();
  frame_state::changed_parents.clear();

  gui::start_frame();
  ImGui::Begin("Viewport");
  hnd::process_input();
  ImGui::End();
  update::refresh_ubos();
}

void end_frame(GLFWwindow *w, uint64_t &begin_time) {
  uint64_t end_time = glfwGetTimerValue();
  gui::end_frame();
  frame_state::last_cpu_frame =
      static_cast<double>(end_time - begin_time) * 1000.f / frame_state::freq;

  glfwSwapBuffers(w);
  glfwPollEvents();
}

void render_window_gui() {
  gui::render_performance_window();
  gui::render_general_settings();
  gui::render_figure_select_gui();
  gui::render_selected_edit_gui();
  gui::render_cursor_gui();
}
void render_viewport() {
  // display
  static auto &fb = framebuffer::get();
  static auto &desc = fb.get_desc();
  ImGui::Begin("Viewport");
  update::setup_globals(ImGui::GetContentRegionAvail());
  const auto s = ImGui::GetContentRegionAvail();

  if (desc.width != s.x || desc.height != s.y) {
    desc.width = s.x;
    desc.height = s.y;
    fb.invalidate();
  }

  // render offscreen
  fb.bind();
  glViewport(0, 0, frame_state::content_area.x, frame_state::content_area.y);
  sys::render_app();
  fb.unbind();

  GLuint t = fb.get_color_att();
  ImGui::Image((void *)(uint64_t)t, s, {0, 1}, {1, 0});

  glm::mat4 gizmo_trans(1.0f);
  sys::get_gizmo_transform(gizmo_trans);
  sys::apply_group_transform(gizmo_trans);

  ImGui::End();
}
} // namespace render
