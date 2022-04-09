#include <registry.h>

namespace ecs {
using r = registry;

ecs::EntityType r::add_entity() {
  static EntityType counter = 0;
  const auto ret = counter++;
  if (counter > 1024)
    throw;
  entities[ret] = 0;
  return ret;
}

void r::delete_entity(ecs::EntityType idx) {
  if (!exists(idx)) {
    return;
  }

  remove_all_components(idx);
  entities.erase(idx);
}

} // namespace ecs
