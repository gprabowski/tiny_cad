#pragma once
#include <ecs.h>
#include <vector>

struct relationship {
  // fields for children
  std::vector<ecs::EntityType> parents;
  std::vector<ecs::EntityType> children;
};
