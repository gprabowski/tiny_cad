#include <cstdlib>
#include <istream>
#include <set>

#include <glad/glad.h>

#define GLM_FORCE_RADIANS
#include <glm/ext/matrix_transform.hpp>
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>

#include <dummy.h>
#include <frame_state.h>
#include <gl_object.h>
#include <log.h>
#include <systems.h>
#include <torus.h>

namespace systems {

void set_vanilla_model_uniform() {
  GLint program;
  glGetIntegerv(GL_CURRENT_PROGRAM, &program);
  const auto model = glm::mat4(1.0f);
  glUniformMatrix4fv(glGetUniformLocation(program, "model"), 1, GL_FALSE,
                     glm::value_ptr(model));
}

void set_model_uniform(const transformation &t) {
  GLint program;
  glGetIntegerv(GL_CURRENT_PROGRAM, &program);

  const auto trans = glm::translate(glm::mat4(1.0f), t.translation);
  const auto scale = glm::scale(glm::mat4(1.0f), t.scale);
  const auto rot = glm::toMat4(glm::quat(glm::radians(t.rotation)));

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
  } else if (g.dmode == gldm::line_strip) {
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glDrawElements(GL_LINE_STRIP, g.indices.size(), GL_UNSIGNED_INT, NULL);
  } else if (g.dmode == gldm::triangles) {
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glDrawElements(GL_TRIANGLES, g.indices.size(), GL_UNSIGNED_INT, NULL);
  }
  glBindVertexArray(0);
  glPointSize(1.0f);
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

void generate_torus_lines(const parametric &p,
                          const std::vector<glm::vec4> &points,
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

void render_figures(
    const std::vector<ecs::EntityType> &selected_parents,
    const std::vector<ecs::EntityType> &selected_primitives,
    const std::vector<ecs::EntityType> &unselected_indices,
    ecs::ComponentStorage<transformation> &transformation_component,
    ecs::ComponentStorage<gl_object> &ogl_component,
    std::shared_ptr<app_state> s, glm::vec3 &center_out) {
  static dummy_point center_of_weight{get_initial_dummy(s->default_program)};

  for (const auto idx : unselected_indices) {
    auto &t = transformation_component[idx];
    auto &gl = ogl_component[idx];
    glUseProgram(gl.program);
    systems::set_model_uniform(t);
    glVertexAttrib4f(1, 0.0f, 0.0f, 1.0f, 1.0f);
    systems::render_points(gl);
  }

  for (const auto idx : selected_primitives) {
    auto &t = transformation_component[idx];
    auto &gl = ogl_component[idx];
    systems::set_model_uniform(t);
    glVertexAttrib4f(1, 1.0f, 0.0f, 0.0f, 1.0f);
    center_of_weight.t.translation += t.translation;
    systems::render_points(gl);
  }

  for (const auto idx : selected_parents) {
    auto &t = transformation_component[idx];
    auto &gl = ogl_component[idx];
    systems::set_model_uniform(t);
    glVertexAttrib4f(1, 1.0f, 0.0f, 0.0f, 1.0f);
    systems::render_points(gl);
  }

  center_of_weight.t.translation *= (1.f / selected_primitives.size());

  glVertexAttrib4f(1, 1.0f, 1.0f, 1.0f, 1.0f);
  systems::set_model_uniform(center_of_weight.t);
  systems::render_points(center_of_weight.g);

  center_out = center_of_weight.t.translation;
  center_of_weight.t.translation = {0.0f, 0.0f, 0.0f};
}
void render_secondary_geometry(ecs::ComponentStorage<secondary_object> &sec,
                               ecs::ComponentStorage<gl_object> &ogl_component,
                               std::shared_ptr<app_state> s) {
  for (const auto &[idx, v] : sec) {
    if (!v.enabled) {
      continue;
    }
    auto &gl = ogl_component[v.val];
    glUseProgram(gl.program);
    systems::set_vanilla_model_uniform();
    glVertexAttrib4f(1, 0.0f, 1.0f, 0.0f, 1.0f);
    systems::render_points(gl);
  }
}

void render_cursors(
    const std::vector<ecs::EntityType> indices,
    const ecs::ComponentStorage<gl_object> &ogl_components,
    ecs::ComponentStorage<transformation> &transformation_component,
    std::shared_ptr<app_state> s) {

  for (auto idx : indices) {
    const auto &gl = ogl_components.at(idx);
    glUseProgram(gl.program);
    glVertexAttrib4f(1, 1.0f, 1.0f, 0.0f, 1.0f);
    auto &t = transformation_component[idx];
    const auto val =
        std::abs((frame_state::view * glm::vec4(t.translation, 1)).z);
    t.scale = glm::vec3(val, val, val);
    systems::set_model_uniform(t);
    systems::render_points(gl);
  }
  glLineWidth(1.0f);
}

void render_app(ecs::component_manager &cm, std::shared_ptr<app_state> s,
                std::vector<ecs::EntityType> &sel,
                std::vector<ecs::EntityType> &unsel,
                std::vector<ecs::EntityType> &changed) {
  static ImVec4 clear_color = ImVec4(0.0f, 0.0f, 0.0f, 1.00f);
  glViewport(0, 0, frame_state::window_w, frame_state::window_h);
  glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w,
               clear_color.z * clear_color.w, clear_color.w);
  glClearDepth(1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glm::vec3 center;
  std::vector<ecs::EntityType> cursors;

  for (auto &[idx, _] : cm.cursor_component) {
    cursors.push_back(idx);
  }

  std::vector<ecs::EntityType> sel_primitives;
  std::vector<ecs::EntityType> sel_parents;

  for (const auto idx : sel) {
    if (!cm.has_component<tag_parent>(idx)) {
      sel_primitives.push_back(idx);
    } else {
      sel_parents.push_back(idx);
    }
  }

  render_figures(sel_parents, sel_primitives, unsel,
                 cm.transformation_components, cm.ogl_components, s, center);

  render_secondary_geometry(cm.secondary_component, cm.ogl_components, s);

  glm::mat4 gizmo_trans(1.0f);
  get_gizmo_transform(gizmo_trans, sel_primitives, cm.transformation_components,
                      s, center);
  apply_group_transform(gizmo_trans, sel_primitives,
                        cm.transformation_components, changed, center);

  render_cursors(cursors, cm.ogl_components, cm.transformation_components, s);
}

void update_changed_relationships(ecs::component_manager &cm,
                                  std::shared_ptr<app_state> &s,
                                  const std::vector<ecs::EntityType> &changed,
                                  const std::vector<ecs::EntityType> &del) {
  std::set<ecs::EntityType> changed_rel;
  for (const auto id : changed) {
    if (cm.has_component<relationship>(id)) {
      auto &rel = cm.get_component<relationship>(id);
      if (rel.parents.size())
        for (auto p : rel.parents) {
          changed_rel.insert(p);
        }
    }
  }

  for (const auto id : del) {
    if (cm.has_component<relationship>(id)) {
      auto &rel = cm.get_component<relationship>(id);
      if (rel.parents.size()) {
        for (auto p : rel.parents) {
          changed_rel.insert(p);
        }
      }
      cm.remove_component<relationship>(id);
    }
  }

  for (const auto p : changed_rel) {
    if (cm.has_component<tag_bezierc>(p) && cm.has_component<relationship>(p)) {
      auto &rel = cm.get_component<relationship>(p);
      auto &gl = cm.get_component<gl_object>(p);
      auto &a = cm.get_component<adaptive>(p);
      auto &sgl = cm.get_component<gl_object>(
          cm.get_component<secondary_object>(p).val);
      regenerate_bezier(rel, a, cm.transformation_components,
                        cm.relationship_component, gl.points, gl.indices,
                        sgl.points, sgl.indices);
      reset_gl_objects(gl);
      reset_gl_objects(sgl);
    }
  }
}

void delete_entities(ecs::component_manager &cm,
                     const std::vector<ecs::EntityType> &del) {
  for (const auto idx : del)
    cm.delete_entity(idx);
}

} // namespace systems
