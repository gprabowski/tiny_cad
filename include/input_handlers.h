#pragma once

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <input_state.h>
#include <memory>
#include <registry.h>

namespace handlers {
void process_input();
}
