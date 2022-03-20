#pragma once

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <app_state.h>
#include <component_manager.h>
#include <ecs.h>
#include <memory>
#include <torus.h>

namespace gui {
void setup_gui(std::shared_ptr<GLFWwindow> &w);
void render_gui();
void show_gui();
void cleanup_gui();
void render_torus_gui(torus_params &tp, parametric &p, gl_object &g,
                      transformation &t);

void render_figure_edit_gui(ecs::component_manager &cm);
void render_figure_select_gui(ecs::component_manager &cm);
void render_selected_edit_gui(ecs::component_manager &cm);
void render_cursor_gui(ecs::component_manager &cm);

} // namespace gui
