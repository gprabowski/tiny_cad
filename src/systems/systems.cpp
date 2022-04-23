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

void regenerate(ecs::EntityType idx) {
  static ecs::registry &reg = ecs::registry::get_registry();
  if (reg.has_component<bspline>(idx)) {
    regenerate_bspline(idx);
  } else if (reg.has_component<bezierc>(idx)) {
    regenerate_bezier(idx);
  } else if (reg.has_component<icurve>(idx)) {
    regenerate_icurve(idx);
  }
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

void render_gl(const gl_object &g) {
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
  } else if (g.dmode == gldm::patches) {
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glPatchParameteri(GL_PATCH_VERTICES, 4);
    glDrawElements(GL_PATCHES, g.indices.size(), GL_UNSIGNED_INT, NULL);
  } else if (g.dmode == gldm::triangles) {
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glDrawElements(GL_TRIANGLES, g.indices.size(), GL_UNSIGNED_INT, NULL);
  }
  glBindVertexArray(0);
  glPointSize(1.0f);
}

void render_visible_entities() {
  auto &reg = ecs::registry::get_registry();

  for (const auto &[idx, _] : reg.get_map<tag_visible>()) {
    auto &t = reg.get_component<transformation>(idx);
    auto &gl = reg.get_component<gl_object>(idx);
    glUseProgram(gl.program);
    systems::set_model_uniform(t);
    glVertexAttrib4f(1, gl.color.r, gl.color.g, gl.color.b, gl.color.a);
    systems::render_gl(gl);
  }
}

void update_center_of_weight() {
  auto &reg = ecs::registry::get_registry();
  auto cen = reg.get_map<tag_center_of_weight>().begin()->first;
  auto &t = reg.get_component<transformation>(cen);
  t.translation = glm::vec3(0.0f);
  auto counter = 0;
  for (auto &[idx, _] : reg.get_map<selected>()) {
    if (!reg.has_component<tag_parent>(idx)) {
      t.translation += reg.get_component<transformation>(idx).translation;
      ++counter;
    }
  }
  t.translation *= (1.f / counter);
  if (counter > 0 && !reg.has_component<tag_visible>(cen)) {
    reg.add_component<tag_visible>(cen, {});
  }
}

void update_cursor() {
  auto &reg = ecs::registry::get_registry();
  auto cur = reg.get_map<cursor_params>().begin()->first;
  auto &t = reg.get_component<transformation>(cur);
  const auto val =
      std::abs((frame_state::view * glm::vec4(t.translation, 1)).z);
  t.scale = glm::vec3(val, val, val);
}

void render_app() {
  static ImVec4 clear_color = ImVec4(0.0f, 0.0f, 0.0f, 1.00f);
  glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w,
               clear_color.z * clear_color.w, clear_color.w);
  glClearDepth(1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  update_center_of_weight();
  update_cursor();
  render_visible_entities();
}

void update_changed_relationships() {
  auto &reg = ecs::registry::get_registry();

  for (const auto id : frame_state::changed) {
    if (reg.has_component<relationship>(id)) {
      auto &rel = reg.get_component<relationship>(id);
      // virtual
      if (reg.has_component<tag_virtual>(id)) {
        auto vparent = rel.parents[0];
        if (reg.has_component<bspline>(vparent)) {
          // 1. find index of virtual point
          auto &prel = reg.get_component<relationship>(vparent);
          const auto vidx =
              std::distance(begin(prel.virtual_children),
                            std::find(begin(prel.virtual_children),
                                      end(prel.virtual_children), id));
          const auto Aidx = prel.children[vidx / 3];
          const auto Bidx = prel.children[vidx / 3 + 1];
          const auto Cidx = prel.children[vidx / 3 + 2];

          auto &a = reg.get_component<transformation>(id);
          auto &A = reg.get_component<transformation>(Aidx);
          auto &B = reg.get_component<transformation>(Bidx);
          auto &C = reg.get_component<transformation>(Cidx);
          auto S = (A.translation + C.translation) / 2.0f;

          switch (vidx % 3) {
          case 0: {
            B.translation = S + 3.f / 2.f * (a.translation - S);
          } break;
          case 1: {
            B.translation =
                C.translation + 3.f / 2.f * (a.translation - C.translation);
          } break;
          case 2: {
            B.translation =
                C.translation + 3.f * (a.translation - C.translation);
          } break;
          }
        }
      }
      // regular
      if (rel.parents.size()) {
        for (auto p : rel.parents) {
          frame_state::changed_parents.insert(p);
        }
      }
      if (rel.children.size()) {
        frame_state::changed_parents.insert(id);
      }
    }
  }

  for (const auto id : frame_state::deleted) {
    if (reg.has_component<relationship>(id)) {
      auto &rel = reg.get_component<relationship>(id);
      if (rel.parents.size()) {
        for (auto p : rel.parents) {
          frame_state::changed_parents.insert(p);
        }
      }
      reg.remove_component<relationship>(id);
    }
  }

  for (const auto p : frame_state::changed_parents) {
    regenerate(p);
  }
}

void delete_entities() {
  auto &reg = ecs::registry::get_registry();

  for (const auto idx : frame_state::deleted)
    reg.delete_entity(idx);
}

} // namespace systems
