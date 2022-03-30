#pragma once

#include <ecs.h>

struct secondary_object {
  ecs::EntityType val;
  bool enabled{false};
};
