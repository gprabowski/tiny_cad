#include <systems.h>

namespace systems {
void add_sel_points_to_parent(ecs::EntityType idx) {
  auto &reg = ecs::registry::get_registry();
  if (!reg.has_component<tag_parent>(idx)) {
    return;
  }
  auto &rel = reg.get_component<relationship>(idx);
  for (const auto &[pidx, _] : reg.get_map<selected>()) {
    if (!reg.has_component<tag_point>(pidx)) {
      continue;
    }
    if (!reg.has_component<relationship>(pidx)) {
      reg.add_component<relationship>(pidx, {});
    }
    auto &prel = reg.get_component<relationship>(pidx);
    rel.children.push_back(pidx);
    prel.parents.push_back(idx);
  }

  if (reg.has_component<bspline>(idx)) {
    regenerate_bspline(idx);
  } else if (reg.has_component<bezierc>(idx)) {
    regenerate_bezier(idx);
  }
}
} // namespace systems
