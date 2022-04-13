#include <algorithm>
#include <cstdio>
#include <iostream>
#include <memory>
#include <tuple>

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <callbacks.h>
#include <constructors.h>
#include <frame_state.h>
#include <framebuffer.h>
#include <gui.h>
#include <init.h>
#include <input_handlers.h>
#include <input_state.h>
#include <log.h>
#include <registry.h>
#include <shader.h>
#include <systems.h>

namespace cst = constructors;
namespace clb = callbacks;
namespace hnd = handlers;
namespace sys = systems;

void get_selected_figure_indices(std::vector<ecs::EntityType> &sel,
                                 std::vector<ecs::EntityType> &unsel) {
  auto &reg = ecs::registry::get_registry();

  for (auto &[idx, _] : reg.get_map<tag_figure>()) {
    if (reg.has_component<selected>(idx))
      sel.push_back(idx);
    else {
      unsel.push_back(idx);
    }
  }
}

void refresh_adaptive(ecs::registry &reg) {}

void setup_globals(const ImVec2 &s) {
  auto &state = input_state::get_input_state();

  int width, height;
  auto w = glfwGetCurrentContext();
  glfwGetFramebufferSize(w, &width, &height);

  auto view = (glm::lookAt(state.cam_pos, state.cam_pos + state.cam_front,
                           state.cam_up));
  auto proj =
      (glm::perspective(45.f, static_cast<float>(s.x) / s.y, 0.1f, 1000.f));

  frame_state::view = view;
  frame_state::proj = proj;
  frame_state::window_h = height;
  frame_state::window_w = width;
  frame_state::content_area = s;
}

void regenererate_adaptive_geometry() {
  auto &reg = ecs::registry::get_registry();

  for (auto &[idx, bez] : reg.get_map<bezierc>()) {
    systems::regenerate_bezier(idx);
  }

  for (auto &[idx, bez] : reg.get_map<bspline>()) {
    systems::regenerate_bspline(idx);
  }
}

void refresh_ubos() {
  glBindBufferBase(GL_UNIFORM_BUFFER, frame_state::common_block_loc,
                   frame_state::common_ubo);
  float *common_ubo_ptr = (float *)glMapNamedBufferRange(
      frame_state::common_ubo, 0, sizeof(float) * 32,
      GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT);
  memcpy(&common_ubo_ptr[0], glm::value_ptr(frame_state::proj),
         sizeof(float) * 16);
  memcpy(&common_ubo_ptr[16], glm::value_ptr(frame_state::view),
         sizeof(float) * 16);
  glUnmapNamedBuffer(frame_state::common_ubo);
  glBindBuffer(GL_UNIFORM_BUFFER, frame_state::common_ubo);
}

void render_fb() {
  // display
  static auto &fb = framebuffer::get();
  static auto &desc = fb.get_desc();
  ImGui::Begin("Viewport");
  ImGui::BeginChild("Render");
  setup_globals(ImGui::GetContentRegionAvail());
  const auto s = ImGui::GetContentRegionAvail();

  if (desc.width != s.x || desc.height != s.y) {
    desc.width = s.x;
    desc.height = s.y;
    fb.invalidate();
  }

  // render offscreen
  fb.bind();
  glViewport(0, 0, s.x, s.y);
  sys::render_app();
  fb.unbind();
  GLuint t = fb.get_color_att();
  ImGui::Image((void *)(uint64_t)t, s, {0, 1}, {1, 0});
  ImGui::End();
}

void main_loop() {
  static glm::vec4 clear_color = {0.0f, 0.0f, 0.0f, 1.00f};
  auto &state = input_state::get_input_state();
  frame_state::freq = glfwGetTimerFrequency();
  auto w = glfwGetCurrentContext();
  std::vector<ecs::EntityType> sel, unsel, parents;
  while (!glfwWindowShouldClose(w)) {
    uint64_t begin_time = glfwGetTimerValue();
    glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w,
                 clear_color.z * clear_color.w, clear_color.w);
    glClearDepth(1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    frame_state::changed.clear();
    frame_state::deleted.clear();
    sel.clear();
    unsel.clear();

    hnd::process_input();

    gui::start_frame();

    render_fb();

    refresh_ubos();
    get_selected_figure_indices(sel, unsel);

    // also renders gizmo

    // make sure fb is the right size

    gui::render_performance_window();
    gui::render_general_settings();
    gui::render_figure_select_gui();
    gui::render_selected_edit_gui();

    gui::render_cursor_gui();
    gui::end_frame();

    sys::update_changed_relationships();
    sys::delete_entities();

    if (state.moved) {
      regenererate_adaptive_geometry();
      state.moved = false;
    }

    uint64_t end_time = glfwGetTimerValue();
    frame_state::last_cpu_frame =
        static_cast<double>(end_time - begin_time) * 1000.f / frame_state::freq;

    glfwSwapBuffers(w);
    glfwPollEvents();
  }
}

int main() {
  log::init();

  auto window = init::init_all("tinyCAD");

  clb::set_keyboard_callback(window);
  clb::set_mouse_callback(window);

  auto &state = input_state::get_input_state();

  state.default_program = shader::LoadProgram("resources/general");
  frame_state::default_program = state.default_program;
  glUseProgram(state.default_program);

  cst::setup_initial_geometry(state.default_program);

  auto &fb = framebuffer::get();
  auto &desc = fb.get_desc();
  desc = {1280, 720};
  fb.invalidate();
  gui::setup_gui(window);
  main_loop();
  gui::cleanup_gui();
}
