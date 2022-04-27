#pragma once

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <ecs.h>
#include <input_state.h>
#include <memory>
#include <registry.h>
#include <torus.h>

namespace gui {

void setup_gui(std::shared_ptr<GLFWwindow> &w);
void start_frame();
void end_frame();
void cleanup_gui();
void render_bezier_gui(tag_figure &fc, gl_object &g, relationship &rel,
                       ecs::EntityType idx);
void render_torus_gui(torus_params &tp, parametric &p, gl_object &g,
                      transformation &t);
void render_icurve_gui(tag_figure &fc, gl_object &g, relationship &rel,
                       ecs::EntityType idx);

void render_performance_window();
void render_general_settings();
void render_figure_select_gui();
void render_selected_edit_gui();

void render_cursor_gui();

} // namespace gui
