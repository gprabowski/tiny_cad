#pragma once

struct cursor_params {
  enum cursor_shape {
    torus = 0,
    point = 1,
    bezierc = 2,
    bspline = 3,
    icurve = 4,
    bsurface = 5
  } current_shape{point};
};
