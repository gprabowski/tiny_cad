#include "constructors.h"
#include <adaptive_score.h>
#include <frame_state.h>
#include <log.h>
#include <shader_manager.h>
#include <systems.h>

namespace systems {

bool regenerate_bspline(ecs::EntityType idx) {
  auto &reg = ecs::registry::get_registry();
  auto &sm = shader_manager::get_manager();

  relationship &r = reg.get_component<relationship>(idx);
  gl_object &g = reg.get_component<gl_object>(idx);
  std::vector<glm::vec4> &out_vertices = g.points;
  std::vector<unsigned int> &out_indices = g.indices;
  auto &bsp = reg.get_component<bspline>(idx);
  gl_object &bgl = reg.get_component<gl_object>(bsp.bezier_polygon);
  std::vector<glm::vec4> &bezier_polygon_vertices = bgl.points;
  std::vector<unsigned int> &bezier_polygon_indices = bgl.indices;
  gl_object &dbgl = reg.get_component<gl_object>(bsp.deboor_polygon);
  std::vector<glm::vec4> &deboor_polygon_vertices = dbgl.points;
  std::vector<unsigned int> &deboor_polygon_indices = dbgl.indices;

  auto &transformations = reg.get_map<transformation>();

  bezier_polygon_vertices.clear();
  bezier_polygon_indices.clear();
  deboor_polygon_vertices.clear();
  deboor_polygon_indices.clear();

  out_vertices.clear();
  out_indices.clear();

  const auto iter_top = r.children.size() >= 4 ? r.children.size() - 3 : 0;

  for (std::size_t i = 0; i < iter_top; ++i) {
    const auto first_child = r.children[i];
    const auto P10 = transformations[first_child].translation;

    const auto second_child = r.children[i + 1];
    const auto P20 = transformations[second_child].translation;

    const auto third_child = r.children[i + 2];
    const auto P30 = transformations[third_child].translation;

    const auto fourth_child = r.children[i + 3];
    const auto P40 = transformations[fourth_child].translation;

    auto tmp = get_pixel_score(P10, P20, P30, P40) / 100;
    auto current_score = std::clamp(tmp, 10.f, 100.f);

    float div = 1.0f / current_score;

    for (float s = 0; s < 1.f; s += div) {
      const float u = 2.f + s;

      const float a41 = (u - 2.0f) / 3.0f;
      const float a31 = (u - 1.0f) / 3.0f;
      const float a21 = (u) / 3.0f;

      const float b41 = 1.0f - a41;
      const float b31 = 1.0f - a31;
      const float b21 = 1.0f - a21;

      const auto P41 = a41 * P40 + b41 * P30;
      const auto P31 = a31 * P30 + b31 * P20;
      const auto P21 = a21 * P20 + b21 * P10;

      const float a42 = (u - 2.0f) / 2.0f;
      const float a32 = (u - 1.0f) / 2.0f;

      const float b42 = 1.0f - a42;
      const float b32 = 1.0f - a32;

      const auto P42 = a42 * P41 + b42 * P31;
      const auto P32 = a32 * P31 + b32 * P21;

      const float a43 = (u - 2.0f) / 1.0f;
      const float b43 = 1.0f - a43;

      const auto P43 = a43 * P42 + b43 * P32;

      out_vertices.push_back(glm::vec4(P43, 1.0f));
    }
  }

  const auto goal_number =
      r.children.size() >= 4 ? (3 * (r.children.size() - 3) + 1) : 0;

  if (r.virtual_children.size() > goal_number) {
    // too many
    const auto diff = r.virtual_children.size() - goal_number;
    for (std::size_t i = 0; i < diff; ++i) {
      const auto cp = r.virtual_children.back();
      reg.remove_component<tag_visible>(cp);
    }
  } else if (r.virtual_children.size() < goal_number) {
    // not enough
    const auto diff = goal_number - r.virtual_children.size();
    for (std::size_t i = 0; i < diff; ++i) {
      const auto cp = constructors::add_virtual_point(
          {}, sm.programs[shader_t::POINT_SHADER].idx);
      auto &crel = reg.get_component<relationship>(cp);
      crel.parents.push_back(idx);
      r.virtual_children.push_back(cp);
    }
  }
  int counter = 0;
  if (goal_number) {
    bool are_visible = reg.has_component<tag_visible>(r.virtual_children[0]);
    for (std::size_t i = 0; i < r.children.size() - 2; ++i) {
      if (i == 0) {
        // add only the connector
        auto &bt1 =
            reg.get_component<transformation>(r.children[i]).translation;
        auto &bt2 =
            reg.get_component<transformation>(r.children[i + 1]).translation;
        auto &bt3 =
            reg.get_component<transformation>(r.children[i + 2]).translation;

        const glm::vec3 pos1 = (1.f / 3.f) * bt1 + (2.f / 3.f) * bt2;
        const glm::vec3 pos2 = (2.f / 3.f) * bt2 + (1.f / 3.f) * bt3;
        auto &t =
            reg.get_component<transformation>(r.virtual_children[counter]);
        t.translation = (pos1 + pos2) / 2.0f;
        if (are_visible) {
          reg.add_component<tag_visible>(r.virtual_children[counter], {});
        }
        ++counter;
      } else {
        auto &bt1 =
            reg.get_component<transformation>(r.children[i]).translation;
        auto &bt2 =
            reg.get_component<transformation>(r.children[i + 1]).translation;
        auto &bt3 =
            reg.get_component<transformation>(r.children[i + 2]).translation;
        // add only the connector
        const glm::vec3 pos1 = (2.f / 3.f) * bt1 + (1.f / 3.f) * bt2;
        const glm::vec3 pos2 = (1.f / 3.f) * bt1 + (2.f / 3.f) * bt2;
        const glm::vec3 pos3 = (2.f / 3.f) * bt2 + (1.f / 3.f) * bt3;
        auto &t1 =
            reg.get_component<transformation>(r.virtual_children[counter]);
        ++counter;
        auto &t2 =
            reg.get_component<transformation>(r.virtual_children[counter]);
        ++counter;
        auto &t3 =
            reg.get_component<transformation>(r.virtual_children[counter]);
        ++counter;

        if (are_visible) {
          reg.add_component<tag_visible>(r.virtual_children[counter - 3], {});
          reg.add_component<tag_visible>(r.virtual_children[counter - 2], {});
          reg.add_component<tag_visible>(r.virtual_children[counter - 1], {});
        }
        t1.translation = pos1;
        t2.translation = pos2;
        t3.translation = (pos2 + pos3) / 2.0f;
      }
    }
  }

  for (auto &c : r.children) {
    auto &t = transformations[c].translation;
    deboor_polygon_vertices.push_back(glm::vec4(t, 1.0f));
  }

  for (auto &c : r.virtual_children) {
    auto &t = transformations[c].translation;
    bezier_polygon_vertices.push_back(glm::vec4(t, 1.0f));
  }

  for (std::size_t i = 0; i < out_vertices.size(); ++i) {
    out_indices.push_back(i);
  }

  for (std::size_t i = 0; i < bezier_polygon_vertices.size(); ++i) {
    bezier_polygon_indices.push_back(i);
  }

  for (std::size_t i = 0; i < deboor_polygon_vertices.size(); ++i) {
    deboor_polygon_indices.push_back(i);
  }

  systems::reset_gl_objects(g);
  systems::reset_gl_objects(bgl);
  systems::reset_gl_objects(dbgl);

  return true;
}
} // namespace systems
