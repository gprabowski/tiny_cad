#include "component_manager.h"
#include "gl_object.h"
#include "parametric.h"
#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "glad/glad.h"

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <cstdio>
#include <memory>
#include <tuple>

#include <app_state.h>
#include <callbacks.h>
#include <gui.h>
#include <init_gl.h>
#include <shader.h>
#include <systems.h>
#include <tags.h>
#include <torus.h>

#include <iostream>

void refresh_common_uniforms(GLuint program, const glm::mat4 &view,
                             const glm::mat4 proj,
                             std::shared_ptr<GLFWwindow> w,
                             std::shared_ptr<app_state> s) {
  glUniformMatrix4fv(glGetUniformLocation(program, "view"), 1, GL_FALSE,
                     glm::value_ptr(view));
  glUniformMatrix4fv(glGetUniformLocation(program, "proj"), 1, GL_FALSE,
                     glm::value_ptr(proj));
}

void render_figures(ecs::component_manager &cm, std::shared_ptr<GLFWwindow> w,
                    std::shared_ptr<app_state> s) {
  GLint last_program;
  glGetIntegerv(GL_CURRENT_PROGRAM, &last_program);

  int display_w, display_h;
  glfwGetFramebufferSize(w.get(), &display_w, &display_h);
  auto proj = glm::perspective(45.f, static_cast<float>(display_w) / display_h,
                               0.1f, 1000.f);
  auto view = glm::lookAt(s->cam_pos, s->cam_pos + s->cam_front, s->cam_up);

  if (last_program != 0) {
    refresh_common_uniforms(last_program, view, proj, w, s);
  }

  for (const auto &[idx, gl] : cm.ogl_components) {
    if (!cm.has_component<tag_figure>(idx)) {
      continue;
    }
    if (last_program != gl.program) {
      last_program = gl.program;
      glUseProgram(gl.program);
      refresh_common_uniforms(gl.program, view, proj, w, s);
    }
    glVertexAttrib4f(1, 1.0f, 1.0f, 0.0f, 1.0f);
    auto &t = cm.get_component<transformation>(idx);
    systems::set_model_uniform(t);
    systems::render_points(gl);
  }
}

void render_cursors(ecs::component_manager &cm, std::shared_ptr<GLFWwindow> w,
                    std::shared_ptr<app_state> s) {
  GLint last_program;

  int display_w, display_h;
  glfwGetFramebufferSize(w.get(), &display_w, &display_h);
  glLineWidth(2.0f);
  auto proj = glm::perspective(45.f, static_cast<float>(display_w) / display_h,
                               0.1f, 1000.f);
  auto view = glm::lookAt(s->cam_pos, s->cam_pos + s->cam_front, s->cam_up);

  glGetIntegerv(GL_CURRENT_PROGRAM, &last_program);
  if (last_program != 0) {
    refresh_common_uniforms(last_program, view, proj, w, s);
  }

  for (const auto &[idx, gl] : cm.ogl_components) {
    if (!cm.has_component<cursor_params>(idx)) {
      continue;
    }
    if (last_program != gl.program) {
      last_program = gl.program;
      glUseProgram(gl.program);
      refresh_common_uniforms(gl.program, view, proj, w, s);
    }
    glVertexAttrib4f(1, 1.0f, 1.0f, 0.0f, 1.0f);
    auto &t = cm.get_component<transformation>(idx);
    const auto val = std::abs((view * glm::vec4(t.translation, 1)).z);
    t.scale = glm::vec3(val, val, val);
    systems::set_model_uniform(t);
    systems::render_points(gl);
  }
  glLineWidth(1.0f);
}

void render_app(ecs::component_manager &cm, std::shared_ptr<GLFWwindow> &w,
                std::shared_ptr<app_state> i) {
  static ImVec4 clear_color = ImVec4(0.0f, 0.0f, 0.0f, 1.00f);
  int display_w, display_h;
  glfwGetFramebufferSize(w.get(), &display_w, &display_h);
  glViewport(0, 0, display_w, display_h);
  glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w,
               clear_color.z * clear_color.w, clear_color.w);
  glClearDepth(1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  render_figures(cm, w, i);
  render_cursors(cm, w, i);
}

ecs::EntityType add_point(ecs::component_manager &cm, transformation &&_t,
                          const GLuint program) {
  const auto p = cm.add_entity();
  cm.add_component<transformation>(p, std::move(_t));
  cm.add_component<gl_object>(p, gl_object{program});
  cm.add_component<tag_figure>(p, tag_figure{});
  cm.add_component<tag_point>(p, tag_point{});

  auto &g = cm.get_component<gl_object>(p);

  g.points.push_back({0.0f, 0.0f, 0.0f, 1.0f});
  g.indices.push_back(0u);
  g.dmode = gl_object::draw_mode::points;

  systems::reset_gl_objects(g);

  return p;
}

ecs::EntityType add_torus(ecs::component_manager &cm, parametric &&_p,
                          transformation &&_t, torus_params &&_tp,
                          const GLuint program) {
  const auto t = cm.add_entity();
  cm.add_component<parametric>(t, std::move(_p));
  cm.add_component<transformation>(t, std::move(_t));
  cm.add_component<gl_object>(t, gl_object{program});
  cm.add_component<torus_params>(t, std::move(_tp));
  cm.add_component<tag_figure>(t, tag_figure{});

  auto &g = cm.get_component<gl_object>(t);
  auto &tp = cm.get_component<torus_params>(t);
  auto &p = cm.get_component<parametric>(t);

  systems::generate_points(tp, p, g.points);
  systems::generate_triangles(p, g.points, g.indices);
  systems::reset_gl_objects(g);

  return t;
}

void process_input(std::shared_ptr<app_state> state,
                   std::shared_ptr<GLFWwindow> w, ecs::component_manager &cm) {
  static float delta_time = 0.0f;
  static float last_frame = 0.0f;

  float currentFrame = glfwGetTime();
  delta_time = currentFrame - last_frame;
  last_frame = currentFrame;

  const float cameraSpeed = 30.f * delta_time; // adjust accordingly
  if (state->pressed[GLFW_KEY_W])
    state->cam_pos += cameraSpeed * state->cam_front;
  if (state->pressed[GLFW_KEY_S])
    state->cam_pos -= cameraSpeed * state->cam_front;
  if (state->pressed[GLFW_KEY_A])
    state->cam_pos -=
        glm::normalize(glm::cross(state->cam_front, state->cam_up)) *
        cameraSpeed;
  if (state->pressed[GLFW_KEY_D])
    state->cam_pos +=
        glm::normalize(glm::cross(state->cam_front, state->cam_up)) *
        cameraSpeed;
  if (state->just_pressed[GLFW_KEY_SPACE]) {
    state->just_pressed = 0b0;
    for (auto &[idx, com] : cm.cursor_component) {
      auto t = cm.get_component<transformation>(idx);
      auto cp = cm.get_component<cursor_params>(idx);
      t.scale = glm::vec3{1.0f, 1.0f, 1.0f};
      if (cp.current_shape == cursor_params::cursor_shape::torus) {
        const auto t1 = add_torus(cm,
                                  parametric{0.0f, 2 * glm::pi<float>(), 0.0f,
                                             2 * glm::pi<float>(), 20u, 50u},
                                  std::move(t), torus_params{1.f, 2.f},
                                  state->default_program);
      } else if (cp.current_shape == cursor_params::cursor_shape::point) {
        const auto p1 = add_point(cm, std::move(t), state->default_program);
      }
    }
  }
}

ecs::EntityType add_cursor(ecs::component_manager &cm, transformation &&_t,
                           gl_object &&_g) {
  const auto t = cm.add_entity();
  cm.add_component<transformation>(t, std::move(_t));
  cm.add_component<gl_object>(t, std::move(_g));
  cm.add_component<cursor_params>(t, cursor_params{});

  auto &g = cm.get_component<gl_object>(t);

  systems::reset_gl_objects(g);

  return t;
}

gl_object get_cursor_geometry(const GLint program) {
  gl_object cursor;
  cursor.points = {{0.0f, 0.0f, 0.0f, 1.0f},  {1.0f, 0.0f, 0.0f, 1.0f},
                   {0.05f, 0.0f, 0.0f, 1.0f}, {1.0f, 0.0f, 0.0f, 1.0f},
                   {0.0f, 0.0f, 0.0f, 1.0f},  {0.0f, 1.0f, 0.0f, 1.0f},
                   {0.0f, 0.05f, 0.0f, 1.0f}, {0.0f, 1.0f, 0.0f, 1.0f},
                   {0.0f, 0.0f, 0.0f, 1.0f},  {0.0f, 0.0f, 1.0f, 1.0f},
                   {0.0f, 0.0f, 0.05f, 1.0f}, {0.0f, 0.0f, 1.0f, 1.0f}};

  cursor.indices = {0, 1, 2, 3, 4, 5};
  cursor.dmode = gl_object::draw_mode::lines;
  cursor.vtype = gl_object::vertex_t::point_color;
  cursor.program = program;

  return cursor;
}

void main_loop(std::shared_ptr<app_state> i, std::shared_ptr<GLFWwindow> w) {
  ecs::component_manager cm;
  glEnable(GL_DEPTH_TEST);
  i->default_program = shader::LoadProgram("resources/general");

  {
    const auto t1 =
        add_torus(cm,
                  parametric{0.0f, 2 * glm::pi<float>(), 0.0f,
                             2 * glm::pi<float>(), 20u, 50u},
                  transformation{}, torus_params{1.f, 2.f}, i->default_program);

    const auto t2 = add_torus(cm,
                              parametric{0.0f, 2 * glm::pi<float>(), 0.0f,
                                         2 * glm::pi<float>(), 20u, 50u},
                              transformation{{40.f, 50.f, 0.f}},
                              torus_params{2.f, 3.f}, i->default_program);

    auto c_gl = get_cursor_geometry(i->default_program);
    const auto c = add_cursor(cm, transformation{}, std::move(c_gl));
  }

  glUseProgram(i->default_program);
  while (!glfwWindowShouldClose(w.get())) {
    process_input(i, w, cm);
    gui::render_gui();
    gui::render_figure_gui(cm);
    gui::render_cursor_gui(cm);
    render_app(cm, w, i);
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
