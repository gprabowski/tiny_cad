#include "gl_object.h"
#include "torus.h"
#include <glad/glad.h>

#define GLM_FORCE_RADIANS
#include <glm/ext/matrix_transform.hpp>
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>

#include <dummy.h>
#include <systems.h>

namespace systems {

void reset_gl_objects(gl_object &g) {
  if (!glIsBuffer(g.vbo)) {
    glCreateBuffers(1, &g.vbo);
  }
  // allocation or reallocation
  glNamedBufferData(g.vbo, sizeof(g.points[0]) * g.points.size(),
                    g.points.data(), GL_DYNAMIC_DRAW);

  if (!glIsBuffer(g.ebo)) {
    glCreateBuffers(1, &g.ebo);
  }
  // allocation or reallocation
  glNamedBufferData(g.ebo, sizeof(unsigned int) * g.indices.size(),
                    g.indices.data(), GL_DYNAMIC_DRAW);

  if (!glIsVertexArray(g.vao)) {
    glCreateVertexArrays(1, &g.vao);
  }

  glVertexArrayVertexBuffer(g.vao, 0, g.vbo, 0, sizeof(g.points[0]));
  glVertexArrayElementBuffer(g.vao, g.ebo);

  switch (g.vtype) {
  case gl_object::vertex_t::point: {
    glBindVertexArray(g.vao);
    glBindVertexBuffer(0, g.vbo, 0, sizeof(glm::vec4));
    glEnableVertexArrayAttrib(g.vao, 0);
    glVertexArrayAttribFormat(g.vao, 0, 4, GL_FLOAT, GL_FALSE, 0);
    glVertexArrayAttribBinding(g.vao, 0, 0);
    glBindVertexArray(0);
  } break;
  case gl_object::vertex_t::point_color: {
    glBindVertexArray(g.vao);
    glBindVertexBuffer(0, g.vbo, 0, 2 * sizeof(glm::vec4));

    glEnableVertexArrayAttrib(g.vao, 0);
    glEnableVertexArrayAttrib(g.vao, 1);

    glVertexArrayAttribFormat(g.vao, 0, 4, GL_FLOAT, GL_FALSE, 0);
    glVertexArrayAttribFormat(g.vao, 1, 4, GL_FLOAT, GL_FALSE,
                              sizeof(glm::vec4));

    glVertexArrayAttribBinding(g.vao, 0, 0);
    glVertexArrayAttribBinding(g.vao, 1, 0);
    glBindVertexArray(0);
  } break;

  default:
    break;
  }
}

void set_model_uniform(const transformation &t) {
  GLint program;
  glGetIntegerv(GL_CURRENT_PROGRAM, &program);

  auto trans = glm::translate(glm::mat4(1.0f), t.translation);
  auto scale = glm::scale(glm::mat4(1.0f), t.scale);
  auto rot = glm::toMat4(glm::quat(glm::radians(t.rotation)));

  const auto model = trans * scale * rot;

  glUniformMatrix4fv(glGetUniformLocation(program, "model"), 1, GL_FALSE,
                     glm::value_ptr(model));
}

void render_points(const gl_object &g) {
  using gldm = gl_object::draw_mode;

  glBindVertexArray(g.vao);
  glPointSize(4.0f);

  if (g.dmode == gldm::points) {
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glDrawArrays(GL_POINTS, 0, g.points.size());
  } else if (g.dmode == gldm::lines) {
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glDrawElements(GL_LINES, g.indices.size(), GL_UNSIGNED_INT, NULL);
  } else if (g.dmode == gldm::triangles) {
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glDrawElements(GL_TRIANGLES, g.indices.size(), GL_UNSIGNED_INT, NULL);
  }
  glBindVertexArray(0);
}

glm::vec4 sample_torus(const torus_params &tp, const float u, const float v) {
  const auto sin_u = sinf(u);
  const auto cos_u = cosf(u);

  const auto sin_v = sinf(v);
  const auto cos_v = cosf(v);

  return {tp.radii[1] * cos_u + tp.radii[0] * cos_u * cos_v,
          tp.radii[1] * sin_u + tp.radii[0] * sin_u * cos_v,
          tp.radii[0] * sin_v, 1.0f};
}

void generate_lines(const parametric &p, const std::vector<glm::vec4> &points,
                    std::vector<unsigned int> &indices) {
  for (int i = 0u; i < p.samples[0]; ++i) {
    for (int j = 0u; j < p.samples[1]; ++j) {
      // add quad as two triangles
      const auto imod = (i + 1) % p.samples[0];
      const auto jmod = (j + 1) % p.samples[1];

      const auto i1 = i * p.samples[1] + j;
      const auto i2 = imod * p.samples[1] + j;
      const auto i3 = imod * p.samples[1] + jmod;
      const auto i4 = i * p.samples[1] + jmod;

      indices.push_back(i1);
      indices.push_back(i2);

      indices.push_back(i2);
      indices.push_back(i3);

      indices.push_back(i3);
      indices.push_back(i4);

      indices.push_back(i4);
      indices.push_back(i1);
    }
  }
}

inline dummy_point get_initial_dummy(GLuint program) {
  transformation t;
  gl_object g;

  g.points = {{0.0f, 0.0f, 0.0f, 1.0f}};
  g.indices = {0u};
  g.dmode = gl_object::draw_mode::points;

  systems::reset_gl_objects(g);

  return dummy_point{std::move(t), std::move(g)};
}

void refresh_common_uniforms(GLuint program, const glm::mat4 &view,
                             const glm::mat4 proj,
                             std::shared_ptr<GLFWwindow> w,
                             std::shared_ptr<app_state> s) {
  glUniformMatrix4fv(glGetUniformLocation(program, "view"), 1, GL_FALSE,
                     glm::value_ptr(view));
  glUniformMatrix4fv(glGetUniformLocation(program, "proj"), 1, GL_FALSE,
                     glm::value_ptr(proj));
}

void decompose(const glm::mat4 &m, glm::vec3 &trans, glm::vec3 &scale,
               glm::vec3 &rot) {
  trans = glm::vec3(m[3]);
  scale = {glm::length(glm::vec3(m[0])), glm::length(glm::vec3(m[1])),
           glm::length(glm::vec3(m[2]))};

  glm::mat4 m_rot(m[0] / scale.x, m[1] / scale.y, m[2] / scale.z,
                  glm::vec4(0.0f, 0.0f, 0.0f, 1.0f));
  rot = glm::degrees(glm::eulerAngles(glm::quat_cast(m_rot)));
}

void render_figures(ecs::component_manager &cm, std::shared_ptr<GLFWwindow> w,
                    std::shared_ptr<app_state> s) {
  static dummy_point center_of_weight{get_initial_dummy(s->default_program)};
  static glm::vec3 prev_scale = {1.0f, 1.0f, 1.0f};
  static bool gizmo_changed = false;

  GLint _last_program;
  glGetIntegerv(GL_CURRENT_PROGRAM, &_last_program);
  GLuint last_program = _last_program;

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

    auto &t = cm.get_component<transformation>(idx);

    if (cm.has_component<selected>(idx)) {
      glVertexAttrib4f(1, 1.0f, 0.0f, 0.0f, 1.0f);
      center_of_weight.t.translation += t.translation;
    } else {
      glVertexAttrib4f(1, 0.0f, 0.0f, 1.0f, 1.0f);
    }
    if (last_program != gl.program) {
      last_program = gl.program;
      glUseProgram(gl.program);
      refresh_common_uniforms(gl.program, view, proj, w, s);
    }
    systems::set_model_uniform(t);
    systems::render_points(gl);
  }

  center_of_weight.t.translation *= (1.f / cm.selected_component.size());
  glVertexAttrib4f(1, 1.0f, 1.0f, 1.0f, 1.0f);
  systems::set_model_uniform(center_of_weight.t);
  systems::render_points(center_of_weight.g);
  // gizmo
  if (cm.selected_component.size()) {
    ImGui::Begin("gizmo");
    ImGuizmo::SetOrthographic(false);
    ImGuizmo::SetDrawlist();
    auto w_w = ImGui::GetWindowWidth();
    auto w_h = ImGui::GetWindowHeight();
    ImGuizmo::SetRect(ImGui::GetWindowPos().x, ImGui::GetWindowPos().y, w_w,
                      w_h);
    auto model =
        glm::translate(glm::mat4(1.0f), center_of_weight.t.translation);

    ImGuizmo::Manipulate(glm::value_ptr(view), glm::value_ptr(proj),
                         s->gizmo_op, ImGuizmo::MODE::WORLD,
                         glm::value_ptr(model));

    if (ImGuizmo::IsUsing()) {
      glm::vec3 trans;
      glm::vec3 rot;
      glm::vec3 scale;

      gizmo_changed = true;

      decompose(model, trans, scale, rot);

      scale = glm::vec3{1.f, 1.f, 1.f} + scale - prev_scale;
      trans = trans - center_of_weight.t.translation;

      auto mtrans = glm::translate(glm::mat4(1.0f), trans);
      auto mscale = glm::scale(glm::mat4(1.0f), scale);
      auto mrot = glm::toMat4(glm::quat(glm::radians(rot)));

      model = mtrans * mscale * mrot;

      prev_scale += scale - glm::vec3{1.0f, 1.0f, 1.0f};

      for (auto &[idx, tmp] : cm.selected_component) {
        auto &t = cm.get_component<transformation>(idx);

        auto trans = glm::translate(
            glm::mat4(1.0f), t.translation - center_of_weight.t.translation);
        auto scale = glm::scale(glm::mat4(1.0f), t.scale);
        auto rot = glm::toMat4(glm::quat(glm::radians(t.rotation)));

        auto tmodel = trans * scale * rot;

        tmodel = model * tmodel;

        decompose(tmodel, t.translation, t.scale, t.rotation);
        t.translation += center_of_weight.t.translation;
      }
    } else if (gizmo_changed == true && !ImGuizmo::IsOver()) {
      gizmo_changed = false;
      prev_scale = {1.0f, 1.0f, 1.0f};
    }
    ImGui::End();
  }

  center_of_weight.t.translation = {0.0f, 0.0f, 0.0f};
}

void render_cursors(ecs::component_manager &cm, std::shared_ptr<GLFWwindow> w,
                    std::shared_ptr<app_state> s) {
  GLint _last_program;
  GLuint last_program;

  int display_w, display_h;
  glfwGetFramebufferSize(w.get(), &display_w, &display_h);
  glLineWidth(2.0f);
  auto proj = glm::perspective(45.f, static_cast<float>(display_w) / display_h,
                               0.1f, 1000.f);
  auto view = glm::lookAt(s->cam_pos, s->cam_pos + s->cam_front, s->cam_up);

  glGetIntegerv(GL_CURRENT_PROGRAM, &_last_program);
  last_program = _last_program;

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
} // namespace systems
