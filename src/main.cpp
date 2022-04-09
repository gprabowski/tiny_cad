#include <algorithm>
#include <cstdio>
#include <iostream>
#include <memory>
#include <tuple>

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <app_state.h>
#include <callbacks.h>
#include <constructors.h>
#include <frame_state.h>
#include <gui.h>
#include <init.h>
#include <input_handlers.h>
#include <log.h>
#include <registry.h>
#include <shader.h>
#include <systems.h>

namespace cst = constructors;
namespace clb = callbacks;
namespace hnd = handlers;
namespace sys = systems;

void get_selected_figure_indices(ecs::registry &reg,
                                 std::vector<ecs::EntityType> &sel,
                                 std::vector<ecs::EntityType> &unsel) {
  for (auto &[idx, _] : reg.get_map<tag_figure>()) {
    if (reg.has_component<selected>(idx))
      sel.push_back(idx);
    else {
      unsel.push_back(idx);
    }
  }
}

void refresh_adaptive(ecs::registry &reg) {}

void setup_globals(std::shared_ptr<app_state> &state) {
  int width, height;
  auto w = glfwGetCurrentContext();
  glfwGetFramebufferSize(w, &width, &height);
  auto view = (glm::lookAt(state->cam_pos, state->cam_pos + state->cam_front,
                           state->cam_up));
  auto proj = (glm::perspective(45.f, static_cast<float>(width) / height, 0.1f,
                                1000.f));
  frame_state::view = view;
  frame_state::proj = proj;
  frame_state::window_h = height;
  frame_state::window_w = width;
}

void regenererate_adaptive_geometry(ecs::registry &reg) {

  for (auto &[idx, _] : reg.get_map<tag_bezierc>()) {
    auto &g = reg.get_component<gl_object>(idx);
    auto &a = reg.get_component<adaptive>(idx);
    auto &sgl = reg.get_component<gl_object>(
        reg.get_component<secondary_object>(idx).val);
    auto &rel = reg.get_component<relationship>(idx);
    systems::regenerate_bezier(rel, a, reg.get_map<transformation>(),
                               reg.get_map<relationship>(), g.points, g.indices,
                               sgl.points, sgl.indices);
    systems::reset_gl_objects(g);
    systems::reset_gl_objects(sgl);
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

void main_loop(ecs::registry &reg, std::shared_ptr<app_state> state) {
  auto w = glfwGetCurrentContext();
  std::vector<ecs::EntityType> sel, unsel, del, parents, changed;
  while (!glfwWindowShouldClose(w)) {

    hnd::process_input(state, reg);
    if (state->moved) {
      regenererate_adaptive_geometry(reg);
      state->moved = false;
    }
    setup_globals(state);
    refresh_ubos();
    gui::start_frame(state);
    get_selected_figure_indices(reg, sel, unsel);

    // also renders gizmo
    sys::render_app(reg, state, sel, unsel, changed);

    gui::render_general_settings(state);
    gui::render_figure_select_gui(reg, del);
    gui::render_selected_edit_gui(reg, state, changed, del);

    gui::render_cursor_gui(reg);
    gui::end_frame();

    sys::update_changed_relationships(reg, state, changed, del);
    sys::delete_entities(reg, del);

    glfwSwapBuffers(w);
    glfwPollEvents();

    sel.clear();
    unsel.clear();
    del.clear();
    changed.clear();
  }
}

int main() {
  log::init();

  auto window = init::init_all("tinyCAD");

  clb::set_keyboard_callback(window);
  clb::set_mouse_callback(window);

  auto state = std::make_shared<app_state>();
  state->default_program = shader::LoadProgram("resources/general");
  glfwSetWindowUserPointer(window.get(), static_cast<void *>(state.get()));
  glUseProgram(state->default_program);

  ecs::registry reg;
  cst::setup_initial_geometry(reg, state->default_program);

  gui::setup_gui(window);
  main_loop(reg, state);
  gui::cleanup_gui();
}
