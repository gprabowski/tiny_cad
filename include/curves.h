#pragma once
#include <ecs.h>

struct bezierc {
    ecs::EntityType bezier_polygon{1u<<31};
};

struct bspline {
    ecs::EntityType bezier_polygon{1u<<31};
    ecs::EntityType deboor_polygon{1u<<31};
};

struct icurve {};
