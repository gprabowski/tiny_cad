#pragma once

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <memory>

namespace callbacks {
void set_keyboard_callback(std::shared_ptr<GLFWwindow> w);
void set_mouse_callback(std::shared_ptr<GLFWwindow> w);
} // namespace callbacks
