#pragma once
#include <ecs.h>
#include <vector>

struct relationship {
  // fields for children
  std::vector<ecs::EntityType> parents;
  std::vector<ecs::EntityType> children;
  std::vector<ecs::EntityType> virtual_children;
  bool indestructible_relation{false};
  int indestructible_counter{0};
};
