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

  const auto bs = entities[idx];
  for (const auto c_t : all_components) {
    if (bs & c_t) {
      remove_component(idx, c_t);
    }
  }

  entities.erase(idx);
}


void cm::remove_component(EntityType idx, ecs::ct cp) {
  // we know that component exists for sure becomes it comes from
  // iteration over enum values which MUST exist, if they don't
  // it's an error in itself
  switch (cp) {
  case ct::OGL_COM: {
    remove_component<gl_object>(idx);
  } break;
  case ct::PARAMETRIC_COM: {
    remove_component<parametric>(idx);
  } break;
  case ct::TAG_CURSOR: {
    remove_component<cursor_params>(idx);
  } break;
  case ct::TAG_FIGURE: {
    remove_component<tag_figure>(idx);
  } break;
  case ct::TAG_POINT: {
    remove_component<tag_point>(idx);
  } break;
  case ct::TORUS_COM: {
    remove_component<torus_params>(idx);
  } break;
  case ct::TRANSFORMATION_COM: {
    remove_component<transformation>(idx);
  } break;
  case ct::TAG_SELECTED: {
    remove_component<selected>(idx);
  } break;
  case ct::TAG_BEZIERC: {
    remove_component<tag_bezierc>(idx);
  }
  case ct::RELATIONSHIP: {
    remove_component<relationship>(idx);
  }
  default:
    return;
  }
}

} // namespace ecs
