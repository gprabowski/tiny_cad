#pragma once

#include <imgui.h>

namespace update {
void setup_globals(const ImVec2 &s);
void regenererate_adaptive_geometry();
void refresh_ubos();
void per_frame_update();
} // namespace update
