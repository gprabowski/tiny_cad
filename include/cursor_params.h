#pragma once

struct cursor_params {
  enum cursor_shape {
    torus = 0,
    point = 1,
    bezierc = 2,
    bspline = 3
  } current_shape{point};
};
