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

void main_loop(ecs::component_manager &cm, std::shared_ptr<app_state> state,
               std::shared_ptr<GLFWwindow> w) {
  while (!glfwWindowShouldClose(w.get())) {
    hnd::process_input(state, w, cm);

    gui::start_frame(state);

    auto sel = sys::render_app(cm, w, state);

    gui::render_general_settings(state);
    // gui::render_figure_edit_gui(cm);
    gui::render_figure_select_gui(cm);
    gui::render_selected_edit_gui(cm, sel);
    gui::render_cursor_gui(cm);
    gui::end_frame();

    sys::update_changed_relationships(cm, sel);

    glfwSwapBuffers(w.get());
    glfwPollEvents();
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
