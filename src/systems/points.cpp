#include <systems.h>

namespace systems {
void add_sel_points_to_parent(ecs::EntityType idx, ecs::component_manager &cm) {
  if (!cm.has_component<tag_parent>(idx)) {
    return;
  }
  auto &rel = cm.get_component<relationship>(idx);
  for (const auto &[pidx, _] : cm.selected_component) {
    if (!cm.has_component<tag_point>(pidx)) {
      continue;
    }
    if (!cm.has_component<relationship>(pidx)) {
      cm.add_component<relationship>(pidx, {});
    }
    auto &prel = cm.get_component<relationship>(pidx);
    rel.children.push_back(pidx);
    prel.parents.push_back(idx);
  }

  auto &g = cm.get_component<gl_object>(idx);
  auto &a = cm.get_component<adaptive>(idx);
  auto &sgl =
      cm.get_component<gl_object>(cm.get_component<secondary_object>(idx).val);
  regenerate_bezier(rel, a, cm.transformation_components,
                    cm.relationship_component, g.points, g.indices, sgl.points,
                    sgl.indices);
  reset_gl_objects(g);
  reset_gl_objects(sgl);
}
} // namespace systems
