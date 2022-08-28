#pragma once

#include <glm/glm.hpp>
#include <imgui.h>

namespace update {
void setup_globals(const ImVec2 &s);
void refresh_ubos();
void refresh_ubos_left();
void refresh_ubos_right();
void per_frame_update();
void refresh_impl(glm::mat4 &col, glm::mat4 &view, glm::mat4 &proj);
void refresh_identity();
} // namespace update
