#pragma once

#include <memory>

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

namespace init_gl {
std::shared_ptr<GLFWwindow> init_screen(const char *caption);
}
