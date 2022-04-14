#pragma once

#include <memory>

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

namespace init {
std::shared_ptr<GLFWwindow> init_all(const char *caption);
void cleanup();
} // namespace init
