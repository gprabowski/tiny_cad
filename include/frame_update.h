#pragma once

#include <imgui.h>

namespace update {
void setup_globals(const ImVec2 &s);
void refresh_ubos();
void refresh_ubos_left();
void refresh_ubos_right();
void per_frame_update();
} // namespace update
