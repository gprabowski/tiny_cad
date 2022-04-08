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

void get_gizmo_transform(
    glm::mat4 &gtrans, std::vector<ecs::EntityType> &indices,
    ecs::ComponentStorage<transformation> &transformation_component,
    std::shared_ptr<app_state> &s, const glm::vec3 &center) {

  ImGui::Begin("gizmo");
  ImGuizmo::SetOrthographic(false);
  ImGuizmo::SetDrawlist();
  auto w_w = ImGui::GetWindowWidth();
  auto w_h = ImGui::GetWindowHeight();
  ImGuizmo::SetRect(ImGui::GetWindowPos().x, ImGui::GetWindowPos().y, w_w, w_h);
  gtrans = glm::translate(glm::mat4(1.0f), center);

  ImGuizmo::Manipulate(glm::value_ptr(frame_state::view),
                       glm::value_ptr(frame_state::proj), s->gizmo_op,
                       ImGuizmo::MODE::WORLD, glm::value_ptr(gtrans));
  ImGui::End();
}

void apply_group_transform(
    glm::mat4 &gtrans, std::vector<ecs::EntityType> &indices,
    ecs::ComponentStorage<transformation> &transformation_component,
    std::vector<ecs::EntityType> &changed, const glm::vec3 &center) {

  static glm::vec3 prev_scale = {1.0f, 1.0f, 1.0f};
  static bool gizmo_changed = false;

  if (ImGuizmo::IsUsing()) {
    glm::vec3 trans;
    glm::vec3 rot;
    glm::vec3 scale;
    changed.insert(changed.end(), indices.begin(), indices.end());
    gizmo_changed = true;
    decompose(gtrans, trans, scale, rot);
    scale = glm::vec3{1.f, 1.f, 1.f} + scale - prev_scale;
    trans = trans - center;
    auto mtrans = glm::translate(glm::mat4(1.0f), trans);
    auto mscale = glm::scale(glm::mat4(1.0f), scale);
    auto mrot = glm::toMat4(glm::quat(glm::radians(rot)));
    gtrans = mtrans * mscale * mrot;
    prev_scale += scale - glm::vec3{1.0f, 1.0f, 1.0f};
    for (const auto idx : indices) {
      auto &t = transformation_component[idx];
      const auto trans =
          glm::translate(glm::mat4(1.0f), t.translation - center);
      const auto scale = glm::scale(glm::mat4(1.0f), t.scale);
      const auto rot = glm::toMat4(glm::quat(glm::radians(t.rotation)));
      const auto tmodel = gtrans * trans * scale * rot;
      decompose(tmodel, t.translation, t.scale, t.rotation);
      t.translation += center;
    }
  } else if (gizmo_changed == true && !ImGuizmo::IsOver()) {
    indices.clear();
    gizmo_changed = false;
    prev_scale = {1.0f, 1.0f, 1.0f};
  }
}
} // namespace systems
