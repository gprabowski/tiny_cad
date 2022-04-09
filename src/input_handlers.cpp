#include <input_handlers.h>

#include <constructors.h>
#include <frame_state.h>
#include <systems.h>

namespace handlers {

inline void add_current_shape_at_cursor(ecs::component_manager &cm,
                                        std::shared_ptr<app_state> &state) {
  auto idx = cm.get_map<cursor_params>().begin()->first;
  ecs::EntityType new_shape;

  auto t = cm.get_component<transformation>(idx);
  const auto &cp = cm.get_component<cursor_params>(idx);
  t.scale = glm::vec3{1.0f, 1.0f, 1.0f};
  if (cp.current_shape == cursor_params::cursor_shape::torus) {
    new_shape = constructors::add_torus(
        cm,
        parametric{
            0.0f, 2 * glm::pi<float>(), 0.0f, 2 * glm::pi<float>(), {20u, 50u}},
        std::move(t), torus_params{1.f, 2.f}, state->default_program);
  } else if (cp.current_shape == cursor_params::cursor_shape::point) {
    new_shape =
        constructors::add_point(cm, std::move(t), state->default_program);
  } else if (cp.current_shape == cursor_params::cursor_shape::bezierc) {
    new_shape = constructors::add_bezier(cm, state, state->default_program);
  }

  if (cm.get_map<selected>().size() == 1) {
    auto s = cm.get_map<selected>().begin()->first;
    if (cm.has_component<tag_parent>(s) &&
        cp.current_shape == cursor_params::cursor_shape::point) {
      cm.add_component<relationship>(new_shape, {{s}, {}});
      auto &srel = cm.get_component<relationship>(s);
      srel.children.push_back(new_shape);
      if (cm.has_component<tag_bezierc>(s)) {
        auto &g = cm.get_component<gl_object>(s);
        auto &sgl = cm.get_component<gl_object>(
            cm.get_component<secondary_object>(s).val);
        auto &a = cm.get_component<adaptive>(s);
        systems::regenerate_bezier(srel, a, cm.get_map<transformation>(),
                                   cm.get_map<relationship>(), g.points,
                                   g.indices, sgl.points, sgl.indices);
        systems::reset_gl_objects(g);
        systems::reset_gl_objects(sgl);
      }
    }
  }
}

void handle_keyboard(std::shared_ptr<app_state> state,
                     ecs::component_manager &cm) {
  static float delta_time = 0.0f;
  static float last_frame = 0.0f;

  float currentFrame = glfwGetTime();
  delta_time = currentFrame - last_frame;
  last_frame = currentFrame;

  if (state->imode == app_state::wasd_mode::camera) {
    const float cameraSpeed = 30.f * delta_time; // adjust accordingly
    if (state->pressed[GLFW_KEY_W]) {
      state->cam_pos += cameraSpeed * state->cam_front;
      state->moved = true;
    }
    if (state->pressed[GLFW_KEY_S]) {
      state->cam_pos -= cameraSpeed * state->cam_front;
      state->moved = true;
    }
    if (state->pressed[GLFW_KEY_A]) {
      state->cam_pos -=
          glm::normalize(glm::cross(state->cam_front, state->cam_up)) *
          cameraSpeed;
      state->moved = true;
    }
    if (state->pressed[GLFW_KEY_D]) {
      state->cam_pos +=
          glm::normalize(glm::cross(state->cam_front, state->cam_up)) *
          cameraSpeed;
      state->moved = true;
    }
  }
  if (state->imode == app_state::wasd_mode::cursor) {
    const float cursor_speed = 10.f * delta_time; // adjust accordingly
    auto &cursor_transform = cm.get_component<transformation>(
        cm.get_map<cursor_params>().begin()->first);
    if (state->pressed[GLFW_KEY_Q])
      cursor_transform.translation -=
          cursor_speed * glm::vec3(0.0f, 0.0f, 1.0f);
    if (state->pressed[GLFW_KEY_E])
      cursor_transform.translation +=
          cursor_speed * glm::vec3(0.0f, 0.0f, 1.0f);
    if (state->pressed[GLFW_KEY_A])
      cursor_transform.translation -=
          cursor_speed * glm::vec3(1.0f, 0.0f, 0.0f);
    if (state->pressed[GLFW_KEY_D])
      cursor_transform.translation +=
          cursor_speed * glm::vec3(1.0f, 0.0f, 0.0f);
    if (state->pressed[GLFW_KEY_S])
      cursor_transform.translation -=
          cursor_speed * glm::vec3(0.0f, 1.0f, 0.0f);
    if (state->pressed[GLFW_KEY_W])
      cursor_transform.translation +=
          cursor_speed * glm::vec3(0.0f, 1.0f, 0.0f);
  }
  if (state->just_pressed[GLFW_KEY_SPACE]) {
    state->just_pressed.reset(GLFW_KEY_SPACE);
    add_current_shape_at_cursor(cm, state);
  }
}

inline bool intersect(const glm::vec3 &ray_orig, const glm::vec3 &dir,
                      const glm::vec3 &sphere_center,
                      const float sphere_radius) {
  const auto L = sphere_center - ray_orig;
  const auto tc = glm::dot(L, dir);
  if (tc <= 0.0) {
    return false;
  }
  const float d2 = glm::dot(L, L) - tc * tc;
  const float radius2 = sphere_radius * sphere_radius;

  return d2 <= radius2;
}

void handle_mouse(std::shared_ptr<app_state> state,
                  ecs::component_manager &cm) {
  if (state->mouse_just_pressed[app_state::mouse_button::left] &&
      state->pressed[GLFW_KEY_LEFT_CONTROL]) {

    state->mouse_just_pressed.reset(app_state::mouse_button::left);

    const auto ndc_x =
        2 *
        (((state->last_mouse.x) / static_cast<float>(frame_state::window_w)) -
         0.5f);
    const auto ndc_y = (-2) * ((state->last_mouse.y) /
                                   static_cast<float>(frame_state::window_h) -
                               0.5f);

    glm::vec4 ndc_dir{ndc_x, ndc_y, -1, 1};

    glm::vec4 world_p = glm::inverse(frame_state::proj) * ndc_dir;
    world_p = world_p / world_p.z;
    world_p = glm::inverse(frame_state::view) * world_p;
    world_p = world_p / world_p.w;

    auto dir = glm::normalize(glm::vec3(world_p) - state->cam_pos);

    for (auto &[idx, t] : cm.get_map<transformation>()) {
      if (cm.has_component<tag_figure>(idx) &&
          intersect(state->cam_pos, dir, t.translation, 1.f)) {
        if (cm.has_component<selected>(idx)) {
          cm.remove_component<selected>(idx);
          return;
        }
        cm.add_component<selected>(idx, {});
        return;
      }
    }

    cm.remove_all<selected>();
  }
}

void process_input(std::shared_ptr<app_state> state,
                   ecs::component_manager &cm) {
  if (ImGui::IsAnyItemActive())
    return;
  handle_keyboard(state, cm);
  handle_mouse(state, cm);
}

} // namespace handlers
