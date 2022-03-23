#pragma once

#include <memory>

#include <glad/glad.h>
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <app_state.h>
#include <component_manager.h>

namespace systems {
void render_figures(ecs::component_manager &cm, std::shared_ptr<GLFWwindow> w,
                    std::shared_ptr<app_state> s);

void render_app(ecs::component_manager &cm, std::shared_ptr<GLFWwindow> &w,
                std::shared_ptr<app_state> i);

void render_cursors(ecs::component_manager &cm, std::shared_ptr<GLFWwindow> w,
                    std::shared_ptr<app_state> s);

void refresh_common_uniforms(GLuint program, const glm::mat4 &view,
                             const glm::mat4 proj,
                             std::shared_ptr<GLFWwindow> w,
                             std::shared_ptr<app_state> s);
} // namespace systems
