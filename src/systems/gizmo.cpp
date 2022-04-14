#define GLM_FORCE_RADIANS
#include <glm/ext/matrix_transform.hpp>
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>

#include <frame_state.h>
#include <systems.h>

namespace systems {
void decompose(const glm::mat4 &m, glm::vec3 &trans, glm::vec3 &scale,
               glm::vec3 &rot) {
  trans = glm::vec3(m[3]);
  scale = {glm::length(glm::vec3(m[0])), glm::length(glm::vec3(m[1])),
           glm::length(glm::vec3(m[2]))};

  glm::mat4 m_rot(m[0] / scale.x, m[1] / scale.y, m[2] / scale.z,
                  glm::vec4(0.0f, 0.0f, 0.0f, 1.0f));
  rot = glm::degrees(glm::eulerAngles(glm::quat_cast(m_rot)));
}

void get_gizmo_transform(glm::mat4 &gtrans) {
  auto &reg = ecs::registry::get_registry();
  auto &is = input_state::get_input_state();

  ImGuizmo::SetOrthographic(false);
  ImGuizmo::SetDrawlist();
  ImGuizmo::SetRect(frame_state::content_pos.x, frame_state::content_pos.y,
                    frame_state::content_area.x, frame_state::content_area.y);
  auto c = reg.get_map<tag_center_of_weight>().begin()->first;
  auto &ct = reg.get_component<transformation>(c);
  gtrans = glm::translate(glm::mat4(1.0f), ct.translation);

  ImGuizmo::Manipulate(glm::value_ptr(frame_state::view),
                       glm::value_ptr(frame_state::proj), is.gizmo_op,
                       ImGuizmo::MODE::WORLD, glm::value_ptr(gtrans));
}

void apply_group_transform(glm::mat4 &gtrans) {
  static glm::vec3 prev_scale = {1.0f, 1.0f, 1.0f};
  static bool gizmo_changed = false;

  auto &reg = ecs::registry::get_registry();

  std::vector<ecs::EntityType> iter;

  if (ImGuizmo::IsUsing()) {
    glm::vec3 trans;
    glm::vec3 rot;
    glm::vec3 scale;
    for (auto &[idx, _] : reg.get_map<selected>()) {
      if (!reg.has_component<tag_parent>(idx)) {
        frame_state::changed.push_back(idx);
        iter.push_back(idx);
      }
    }
    gizmo_changed = true;
    decompose(gtrans, trans, scale, rot);
    scale = glm::vec3{1.f, 1.f, 1.f} + scale - prev_scale;
    auto c = reg.get_map<tag_center_of_weight>().begin()->first;
    auto &ct = reg.get_component<transformation>(c);
    trans = trans - ct.translation;
    auto mtrans = glm::translate(glm::mat4(1.0f), trans);
    auto mscale = glm::scale(glm::mat4(1.0f), scale);
    auto mrot = glm::toMat4(glm::quat(glm::radians(rot)));
    gtrans = mtrans * mscale * mrot;
    prev_scale += scale - glm::vec3{1.0f, 1.0f, 1.0f};
    for (const auto idx : iter) {
      auto &t = reg.get_component<transformation>(idx);
      const auto trans =
          glm::translate(glm::mat4(1.0f), t.translation - ct.translation);
      const auto scale = glm::scale(glm::mat4(1.0f), t.scale);
      const auto rot = glm::toMat4(glm::quat(glm::radians(t.rotation)));
      const auto tmodel = gtrans * trans * scale * rot;
      decompose(tmodel, t.translation, t.scale, t.rotation);
      t.translation += ct.translation;
    }
  } else if (gizmo_changed == true && !ImGuizmo::IsOver()) {
    gizmo_changed = false;
    prev_scale = {1.0f, 1.0f, 1.0f};
  }
}
} // namespace systems
