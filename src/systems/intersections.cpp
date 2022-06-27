#include "ecs.h"
#include <registry.h>
#include <systems.h>

namespace systems {
// general case solver
template <typename P_t, typename Q_t>
ecs::EntityType find_intersection(P_t Pfunc, Q_t Qfunc) {
  const auto tmp1 = Pfunc(0.0, 0.0);
  const auto tmp2 = Qfunc(0.0, 0.0);

  if (glm::distance(tmp1, tmp2) < 1.0f) {
    return ecs::null_entity;
  } else {
    return ecs::null_entity;
  }
}

ecs::EntityType intersect(sampler &first, sampler &second) {
  return ecs::null_entity;
}

} // namespace systems
