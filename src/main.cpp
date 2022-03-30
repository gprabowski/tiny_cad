#include <algorithm>
#include <cstdio>
#include <iostream>
#include <memory>
#include <tuple>

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <app_state.h>
#include <callbacks.h>
#include <component_manager.h>
#include <constructors.h>
#include <frame_state.h>
#include <gui.h>
#include <init.h>
#include <input_handlers.h>
#include <log.h>
#include <shader.h>
#include <systems.h>

namespace cst = constructors;
namespace clb = callbacks;
namespace hnd = handlers;
namespace sys = systems;

void get_selected_figure_indices(ecs::component_manager &cm,
                                 std::vector<ecs::EntityType> &sel,
                                 std::vector<ecs::EntityType> &unsel) {
  for (auto &[idx, _] : cm.figure_component) {
    if (cm.has_component<selected>(idx))
      sel.push_back(idx);
    else {
      unsel.push_back(idx);
    }
  }
}

void refresh_adaptive(ecs::component_manager &cm) {}

void setup_globals(std::shared_ptr<app_state> &state,
                   std::shared_ptr<GLFWwindow> &w) {
  int width, height;
  glfwGetWindowSize(w.get(), &width, &height);
  auto view = (glm::lookAt(state->cam_pos, state->cam_pos + state->cam_front,
                           state->cam_up));
  auto proj = (glm::perspective(45.f, static_cast<float>(width) / height, 0.1f,
                                1000.f));
  frame_state::view = view;
  frame_state::proj = proj;
  frame_state::window_h = height;
  frame_state::window_w = width;
}

void regenererate_adaptive_geometry(ecs::component_manager &cm) {

  for (auto &[idx, _] : cm.bezierc_component) {
    auto &g = cm.get_component<gl_object>(idx);
    auto &a = cm.get_component<adaptive>(idx);
    auto &sgl = cm.get_component<gl_object>(
        cm.get_component<secondary_object>(idx).val);
    auto &rel = cm.get_component<relationship>(idx);
    systems::regenerate_bezier(rel, a, cm.transformation_components,
                               cm.relationship_component, g.points, g.indices,
                               sgl.points, sgl.indices);
    systems::reset_gl_objects(g);
    systems::reset_gl_objects(sgl);
  }
}

void main_loop(ecs::component_manager &cm, std::shared_ptr<app_state> state,
               std::shared_ptr<GLFWwindow> w) {
  std::vector<ecs::EntityType> sel, unsel, del, parents, changed;
  while (!glfwWindowShouldClose(w.get())) {

    hnd::process_input(state, w, cm);
    if (state->moved) {
      regenererate_adaptive_geometry(cm);
      state->moved = false;
    }
    setup_globals(state, w);

    gui::start_frame(state);

    get_selected_figure_indices(cm, sel, unsel);

    // also renders gizmo
    sys::render_app(cm, w, state, sel, unsel, changed);

    gui::render_general_settings(state);
    gui::render_figure_select_gui(cm, del);
    gui::render_selected_edit_gui(cm, state, changed, del);

    gui::render_cursor_gui(cm);
    gui::end_frame();

    sys::update_changed_relationships(cm, state, changed, del);
    sys::delete_entities(cm, del);

    glfwSwapBuffers(w.get());
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

  ecs::component_manager cm;
  cst::setup_initial_geometry(cm, state->default_program);

  gui::setup_gui(window);
  main_loop(cm, state, window);
  gui::cleanup_gui();
}
