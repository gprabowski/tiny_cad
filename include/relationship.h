#pragma once
#include <ecs.h>

struct relationship {
  // fields for children
  ecs::EntityType parent{ecs::null_entity};
  ecs::EntityType next_child{ecs::null_entity};
  ecs::EntityType prev_child{ecs::null_entity};

  // fields for parents
  ecs::EntityType first_child{ecs::null_entity};
  std::size_t num_children{0};
};
