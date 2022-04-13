#include <input_handlers.h>

#include <constructors.h>
#include <frame_state.h>
#include <systems.h>

namespace handlers {

inline void add_current_shape_at_cursor() {
  auto &reg = ecs::registry::get_registry();
  auto &state = input_state::get_input_state();

  auto idx = reg.get_map<cursor_params>().begin()->first;
  ecs::EntityType new_shape;

  auto t = reg.get_component<transformation>(idx);
  const auto &cp = reg.get_component<cursor_params>(idx);
  t.scale = glm::vec3{1.0f, 1.0f, 1.0f};
  if (cp.current_shape == cursor_params::cursor_shape::torus) {
    new_shape = constructors::add_torus(
        parametric{
            0.0f, 2 * glm::pi<float>(), 0.0f, 2 * glm::pi<float>(), {20u, 50u}},
        std::move(t), torus_params{1.f, 2.f}, state.default_program);
  } else if (cp.current_shape == cursor_params::cursor_shape::point) {
    new_shape = constructors::add_point(std::move(t), state.default_program);
  } else if (cp.current_shape == cursor_params::cursor_shape::bezierc) {
    new_shape = constructors::add_bezier(state.default_program);
  } else if (cp.current_shape == cursor_params::cursor_shape::bspline) {
    new_shape = constructors::add_bspline(state.default_program);
  }

  if (reg.get_map<selected>().size() == 1) {
    auto s = reg.get_map<selected>().begin()->first;
    if (reg.has_component<tag_parent>(s) &&
        cp.current_shape == cursor_params::cursor_shape::point) {
      reg.add_component<relationship>(new_shape, {{s}, {}});
      auto &srel = reg.get_component<relationship>(s);
      srel.children.push_back(new_shape);
      if (reg.has_component<bezierc>(s)) {
        systems::regenerate_bezier(s);
      } else if (reg.has_component<bspline>(s)) {
        systems::regenerate_bspline(s);
      }
    }
  }
}

void handle_keyboard() {
  static float delta_time = 0.0f;
  static float last_frame = 0.0f;

  auto &reg = ecs::registry::get_registry();
  auto &state = input_state::get_input_state();

  float currentFrame = glfwGetTime();
  delta_time = currentFrame - last_frame;
  last_frame = currentFrame;

  if (state.imode == input_state::wasd_mode::camera) {
    const float cameraSpeed = 30.f * delta_time; // adjust accordingly
    if (state.pressed[GLFW_KEY_W]) {
      state.cam_pos += cameraSpeed * state.cam_front;
      state.moved = true;
    }
    if (state.pressed[GLFW_KEY_S]) {
      state.cam_pos -= cameraSpeed * state.cam_front;
      state.moved = true;
    }
    if (state.pressed[GLFW_KEY_A]) {
      state.cam_pos -=
          glm::normalize(glm::cross(state.cam_front, state.cam_up)) *
          cameraSpeed;
      state.moved = true;
    }
    if (state.pressed[GLFW_KEY_D]) {
      state.cam_pos +=
          glm::normalize(glm::cross(state.cam_front, state.cam_up)) *
          cameraSpeed;
      state.moved = true;
    }
  }
  if (state.imode == input_state::wasd_mode::cursor) {
    const float cursor_speed = 10.f * delta_time; // adjust accordingly
    auto &cursor_transform = reg.get_component<transformation>(
        reg.get_map<cursor_params>().begin()->first);
    if (state.pressed[GLFW_KEY_Q])
      cursor_transform.translation -=
          cursor_speed * glm::vec3(0.0f, 0.0f, 1.0f);
    if (state.pressed[GLFW_KEY_E])
      cursor_transform.translation +=
          cursor_speed * glm::vec3(0.0f, 0.0f, 1.0f);
    if (state.pressed[GLFW_KEY_A])
      cursor_transform.translation -=
          cursor_speed * glm::vec3(1.0f, 0.0f, 0.0f);
    if (state.pressed[GLFW_KEY_D])
      cursor_transform.translation +=
          cursor_speed * glm::vec3(1.0f, 0.0f, 0.0f);
    if (state.pressed[GLFW_KEY_S])
      cursor_transform.translation -=
          cursor_speed * glm::vec3(0.0f, 1.0f, 0.0f);
    if (state.pressed[GLFW_KEY_W])
      cursor_transform.translation +=
          cursor_speed * glm::vec3(0.0f, 1.0f, 0.0f);
  }
  if (state.just_pressed[GLFW_KEY_SPACE]) {
    state.just_pressed.reset(GLFW_KEY_SPACE);
    add_current_shape_at_cursor();
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

void handle_mouse() {
  auto &reg = ecs::registry::get_registry();
  auto &state = input_state::get_input_state();

  if (state.mouse_just_pressed[input_state::mouse_button::left] &&
      state.pressed[GLFW_KEY_LEFT_CONTROL]) {

    state.mouse_just_pressed.reset(input_state::mouse_button::left);

    const auto ndc_x =
        2 *
        (((state.last_mouse.x) / static_cast<float>(frame_state::window_w)) -
         0.5f);
    const auto ndc_y = (-2) * ((state.last_mouse.y) /
                                   static_cast<float>(frame_state::window_h) -
                               0.5f);

    glm::vec4 ndc_dir{ndc_x, ndc_y, -1, 1};

    glm::vec4 world_p = glm::inverse(frame_state::proj) * ndc_dir;
    world_p = world_p / world_p.z;
    world_p = glm::inverse(frame_state::view) * world_p;
    world_p = world_p / world_p.w;

    auto dir = glm::normalize(glm::vec3(world_p) - state.cam_pos);

    for (auto &[idx, _] : reg.get_map<tag_clickable>()) {
      auto &t = reg.get_component<transformation>(idx);
      if (intersect(state.cam_pos, dir, t.translation, 1.f)) {
        if (reg.has_component<selected>(idx)) {
          reg.remove_component<selected>(idx);
          return;
        }

        if (reg.has_component<tag_virtual>(idx) ||
            (reg.get_map<selected>().size() &&
             reg.has_component<tag_virtual>(
                 reg.get_map<selected>().begin()->first))) {
          reg.remove_all<selected>();
        }
        reg.add_component<selected>(idx, {});

        return;
      }
    }

    reg.remove_all<selected>();
  }
}

void process_input() {
  if (ImGui::IsAnyItemActive())
    return;
  handle_keyboard();
  handle_mouse();
}

} // namespace handlers
