#pragma once

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <app_state.h>
#include <component_manager.h>
#include <memory>

namespace handlers {
void process_input(std::shared_ptr<app_state> state,
                   ecs::component_manager &cm);
}
