#include <algorithm>
#include <callbacks.h>
#include <cstdio>
#include <imgui.h>
#include <input_state.h>
#include <memory>

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

static void key_callback(GLFWwindow *window, int key, int scancode, int action,
                         int mods) {
  auto &s = input_state::get_input_state();

  if (action == GLFW_PRESS) {
    s.just_pressed.set(static_cast<size_t>(key));
    s.pressed.set(static_cast<size_t>(key));
  } else if (action == GLFW_RELEASE) {
    s.just_pressed.reset(static_cast<size_t>(key));
    s.pressed.reset(static_cast<size_t>(key));
  }
}

inline input_state::mouse_button mbutton_glfw_to_enum(int glfw_mbutton) {
  if (glfw_mbutton == GLFW_MOUSE_BUTTON_LEFT) {
    return input_state::mouse_button::left;
  } else if (glfw_mbutton == GLFW_MOUSE_BUTTON_RIGHT) {
    return input_state::mouse_button::right;
  } else if (glfw_mbutton == GLFW_MOUSE_BUTTON_MIDDLE) {
    return input_state::mouse_button::middle;
  }

  return input_state::mouse_button::other;
}

void mouse_button_callback(GLFWwindow *w, int button, int action, int mods) {

  auto &s = input_state::get_input_state();
  double xpos, ypos;
  glfwGetCursorPos(w, &xpos, &ypos);

  if (action == GLFW_PRESS) {
    s.last_mouse = {xpos, ypos};
    const auto pos = mbutton_glfw_to_enum(button);
    s.mouse_just_pressed.set(pos);
    s.mouse_pressed.set(pos);
  }

  if (action == GLFW_RELEASE) {
    const auto pos = mbutton_glfw_to_enum(button);
    s.last_mouse = {xpos, ypos};
    s.mouse_just_pressed.reset(pos);
    s.mouse_pressed.reset(pos);
  }
}

void reorient_camera(double xpos, double ypos) {
  auto &s = input_state::get_input_state();

  float xoffset = xpos - s.last_mouse.x;
  float yoffset = s.last_mouse.y -
                  ypos; // reversed since y-coordinates range from bottom to top
  s.last_mouse = {xpos, ypos};

  const float sensitivity = 0.1f;
  xoffset *= sensitivity;
  yoffset *= sensitivity;

  s.yaw += xoffset;
  s.pitch = std::clamp<float>(s.pitch + yoffset, -90, 90);
  glm::vec3 direction;
  direction.x = cos(glm::radians(s.yaw)) * cos(glm::radians(s.pitch));
  direction.y = sin(glm::radians(s.pitch));
  direction.z = sin(glm::radians(s.yaw)) * cos(glm::radians(s.pitch));
  s.cam_front = glm::normalize(direction);
}

void mouse_move_callback(GLFWwindow *w, double xpos, double ypos) {
  auto &s = input_state::get_input_state();

  if (s.mouse_pressed[input_state::mouse_button::right]) {
    reorient_camera(xpos, ypos);
  }
}

namespace callbacks {

void set_keyboard_callback(std::shared_ptr<GLFWwindow> w) {
  glfwSetKeyCallback(w.get(), key_callback);
}

void set_mouse_callback(std::shared_ptr<GLFWwindow> w) {
  glfwSetMouseButtonCallback(w.get(), mouse_button_callback);
  glfwSetCursorPosCallback(w.get(), mouse_move_callback);
}

} // namespace callbacks
