#pragma once

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <app_state.h>
#include <memory>
#include <registry.h>

namespace handlers {
void process_input(std::shared_ptr<app_state> state, ecs::registry &reg);
}
