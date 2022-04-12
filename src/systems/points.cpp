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

  auto &g = reg.get_component<gl_object>(idx);
  auto &a = reg.get_component<adaptive>(idx);
  auto &sgl = reg.get_component<gl_object>(
      reg.get_component<secondary_object>(idx).val);
  if (reg.has_component<tag_bspline>(idx)) {
    regenerate_bspline(idx, rel, a, g.points, g.indices, sgl.points,
                       sgl.indices);

  } else if (reg.has_component<tag_bezierc>(idx)) {
    regenerate_bezier(rel, a, g.points, g.indices, sgl.points, sgl.indices);
  }
  reset_gl_objects(g);
  reset_gl_objects(sgl);
}
} // namespace systems
