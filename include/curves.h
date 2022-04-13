#pragma once
#include <ecs.h>

struct bezierc {
  ecs::EntityType bezier_polygon;
};
struct bspline {
  ecs::EntityType bezier_polygon;
  ecs::EntityType deboor_polygon;
};
