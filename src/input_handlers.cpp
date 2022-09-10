#include <input_handlers.h>

#include <constructors.h>
#include <frame_state.h>
#include <shader_manager.h>
#include <systems.h>

#include <iostream>

namespace handlers {

inline void add_current_shape_at_cursor() {
  auto &reg = ecs::registry::get_registry();
  auto &sm = shader_manager::get_manager();

  auto idx = reg.get_map<cursor_params>().begin()->first;
  ecs::EntityType new_shape;

  auto t = reg.get_component<transformation>(idx);
  const auto &cp = reg.get_component<cursor_params>(idx);
  t.scale = glm::vec3{1.0f, 1.0f, 1.0f};
  if (cp.current_shape == cursor_params::cursor_shape::torus) {
    new_shape = constructors::add_torus(
        parametric{
            0.0f, 2 * glm::pi<float>(), 0.0f, 2 * glm::pi<float>(), {20u, 50u}},
        std::move(t), torus_params{2.f, 4.f},
        sm.programs[shader_t::TORUS_SHADER].idx);
  } else if (cp.current_shape == cursor_params::cursor_shape::point) {
    new_shape = constructors::add_point(
        std::move(t), sm.programs[shader_t::POINT_SHADER].idx);
  } else if (cp.current_shape == cursor_params::cursor_shape::bezierc &&
             reg.get_map<selected>().size() >= 2) {
    new_shape = constructors::add_bezier(
        sm.programs[shader_t::BEZIER_CURVE_SHADER].idx);
  } else if (cp.current_shape == cursor_params::cursor_shape::bspline &&
             reg.get_map<selected>().size() >= 2) {
    new_shape = constructors::add_bspline(
        sm.programs[shader_t::BSPLINE_CURVE_SHADER].idx);
  } else if (cp.current_shape == cursor_params::cursor_shape::icurve &&
             reg.get_map<selected>().size() > 2) {
    new_shape = constructors::add_icurve(
        sm.programs[shader_t::INTERPOLATION_CURVE_SHADER].idx);
  } else if (cp.current_shape == cursor_params::cursor_shape::bsurface) {
    new_shape = constructors::add_bezier_surface_builder(
        std::move(t), sm.programs[shader_t::GENERAL_SHADER].idx);
  } else if (cp.current_shape == cursor_params::cursor_shape::bspsurface) {
    new_shape = constructors::add_bspline_surface_builder(
        std::move(t), sm.programs[shader_t::GENERAL_SHADER].idx);
  } else if (cp.current_shape == cursor_params::cursor_shape::gregory) {
    new_shape =
        constructors::add_gregory(sm.programs[shader_t::GENERAL_SHADER].idx);
  } else if (cp.current_shape == cursor_params::cursor_shape::intersection) {
    new_shape = constructors::add_intersection(
        sm.programs[shader_t::GENERAL_SHADER].idx);
  }

  if (reg.get_map<selected>().size() == 1) {
    auto s = reg.get_map<selected>().begin()->first;
    if (reg.has_component<tag_parent>(s) &&
        cp.current_shape == cursor_params::cursor_shape::point) {
      reg.add_component<relationship>(new_shape, {{s}, {}});
      auto &srel = reg.get_component<relationship>(s);
      srel.children.push_back(new_shape);
      systems::regenerate(s);
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
    }
    if (state.pressed[GLFW_KEY_S]) {
      state.cam_pos -= cameraSpeed * state.cam_front;
    }
    if (state.pressed[GLFW_KEY_A]) {
      state.cam_pos -=
          glm::normalize(glm::cross(state.cam_front, state.cam_up)) *
          cameraSpeed;
    }
    if (state.pressed[GLFW_KEY_D]) {
      state.cam_pos +=
          glm::normalize(glm::cross(state.cam_front, state.cam_up)) *
          cameraSpeed;
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

inline glm::vec2 get_ndc(const glm::vec2 &v) {
  return {2 * (((v.x - frame_state::content_pos.x) /
                static_cast<float>(frame_state::content_area.x)) -
               0.5f),
          -2 * (((v.y - frame_state::content_pos.y) /
                 static_cast<float>(frame_state::content_area.y)) -
                0.5f)};
}

inline bool is_between(const glm::vec2 &p, const glm::vec2 st,
                       const glm::vec2 end) {
  const auto is_bet = [](float p, float a, float b) {
    return ((p >= a && p <= b) || (p >= b && p <= a));
  };
  return is_bet(p.x, st.x, end.x) && is_bet(p.y, st.y, end.y);
}

void handle_mouse() {
  auto &reg = ecs::registry::get_registry();
  auto &state = input_state::get_input_state();

  if (state.mouse_just_pressed[input_state::mouse_button::left] &&
      state.pressed[GLFW_KEY_LEFT_CONTROL] && !state.mouse_selecting) {
    state.selection_start = state.last_mouse;
    state.mouse_selecting = true;
    const auto sel_id = reg.get_map<tags_selection_rect>().begin()->first;
    auto &tr = reg.get_component<transformation>(sel_id);
    tr.scale = {0.f, 0.f, 0.f};
  } else if (state.mouse_selecting &&
             !state.mouse_pressed[input_state::mouse_button::left]) {
    state.mouse_selecting = false;

    state.mouse_just_pressed.reset(input_state::mouse_button::left);

    auto ndc_last = get_ndc({state.last_mouse.x, state.last_mouse.y});
    auto ndc_start =
        get_ndc({state.selection_start.x, state.selection_start.y});

    int added{0};
    for (auto &[idx, _] : reg.get_map<tag_clickable>()) {
      auto &t = reg.get_component<transformation>(idx);
      glm::vec4 pos = frame_state::proj * frame_state::view *
                      glm::vec4{t.translation, 1.0f};
      pos /= pos.w;
      if (is_between({pos.x, pos.y}, ndc_start, ndc_last)) {
        ++added;
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
      }
    }

    if (added == 0) {
      reg.remove_all<selected>();
    }
  }
}

void process_input() {
  using fs = frame_state;
  if (!ImGui::IsMouseHoveringRect(fs::content_pos,
                                  {fs::content_pos.x + fs::content_area.x,
                                   fs::content_pos.y + fs::content_area.y})) {
    return;
  }
  handle_keyboard();
  handle_mouse();
}

} // namespace handlers
