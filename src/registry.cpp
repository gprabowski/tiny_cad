#include <registry.h>

namespace ecs {
using r = registry;

ecs::EntityType r::add_entity() {
  static EntityType counter = 0;
  const auto ret = counter++;
  entities[ret] = 0;
  return ret;
}

void r::delete_entity(ecs::EntityType idx) {
  if (!exists(idx)) {
    return;
  }

  if (has_component<relationship>(idx)) {
    auto &r = get_component<relationship>(idx);
    if (r.indestructible_counter > 0) {
      return;
    }
  }

  remove_all_components(idx);
  entities.erase(idx);
}

} // namespace ecs
