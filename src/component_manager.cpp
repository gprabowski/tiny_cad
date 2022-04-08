#include <component_manager.h>

namespace ecs {
using cm = component_manager;

ecs::EntityType cm::add_entity() {
  static EntityType counter = 0;
  const auto ret = counter++;
  if (counter > 1024)
    throw;
  entities[ret] = 0;
  return ret;
}

void cm::delete_entity(ecs::EntityType idx) {
  if (!exists(idx)) {
    return;
  }

  remove_all_components(idx);
  entities.erase(idx);
}

} // namespace ecs
