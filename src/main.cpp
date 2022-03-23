#include "component_manager.h"
#include "gl_object.h"
#include "parametric.h"

#include "glad/glad.h"

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

// order is important here
// as imguizmo depends on imgui
#include <imgui.h>

#define USE_IMGUI_API
#include <ImGuizmo.h>

#include <algorithm>
#include <cstdio>
#include <memory>
#include <tuple>

#include <app_state.h>
#include <callbacks.h>
#include <constructors.h>
#include <dummy.h>
#include <gui.h>
#include <init_gl.h>
#include <input_handlers.h>
#include <rendering_systems.h>
#include <shader.h>
#include <systems.h>
#include <tags.h>
#include <torus.h>

#include <iostream>

void main_loop(std::shared_ptr<app_state> i, std::shared_ptr<GLFWwindow> w) {
  ecs::component_manager cm;
  glEnable(GL_DEPTH_TEST);
  i->default_program = shader::LoadProgram("resources/general");

  auto c_gl = constructors::get_cursor_geometry(i->default_program);
  const auto c =
      constructors::add_cursor(cm, transformation{}, std::move(c_gl));
  auto &cursor_transf = cm.get_component<transformation>(c);

  glUseProgram(i->default_program);
  while (!glfwWindowShouldClose(w.get())) {
    handlers::process_input(i, w, cm, cursor_transf);
    gui::render_gui(i);
    gui::render_figure_edit_gui(cm);
    gui::render_figure_select_gui(cm);
    gui::render_selected_edit_gui(cm);
    gui::render_cursor_gui(cm);
    systems::render_app(cm, w, i);

    gui::show_gui();
    glfwSwapBuffers(w.get());
    glfwPollEvents();
  }
}

int main() {
  auto state = std::make_shared<app_state>();
  auto window = init_gl::init_screen("GLFW test");

  glfwSetWindowUserPointer(window.get(), static_cast<void *>(state.get()));

  callbacks::set_keyboard_callback(window);
  callbacks::set_mouse_callback(window);

  gui::setup_gui(window);
  main_loop(state, window);
  gui::cleanup_gui();
}
