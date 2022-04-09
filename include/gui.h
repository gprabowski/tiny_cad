#pragma once

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <app_state.h>
#include <ecs.h>
#include <memory>
#include <registry.h>
#include <torus.h>

namespace gui {

enum class point_action { none, edit, del };

void setup_gui(std::shared_ptr<GLFWwindow> &w);
void start_frame(std::shared_ptr<app_state> &s);
void end_frame();
void cleanup_gui();
void render_bezier_gui(ecs::registry &reg, tag_figure &fc, gl_object &g,
                       relationship &rel, ecs::EntityType idx);
void render_torus_gui(torus_params &tp, parametric &p, gl_object &g,
                      transformation &t);

void render_general_settings(std::shared_ptr<app_state> &s);
point_action render_figure_select_gui(ecs::registry &reg,
                                      std::vector<ecs::EntityType> &deleted);
void render_selected_edit_gui(ecs::registry &reg, std::shared_ptr<app_state> &s,
                              std::vector<ecs::EntityType> &changed,
                              std::vector<ecs::EntityType> &deleted);

void render_cursor_gui(ecs::registry &reg);

} // namespace gui
