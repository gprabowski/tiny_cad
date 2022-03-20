#pragma once

struct cursor_params {
  enum cursor_shape { torus = 0, point = 1 } current_shape{torus};
};
