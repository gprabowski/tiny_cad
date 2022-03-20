#include <component_manager.h>

namespace ecs {
using cm = component_manager;

ecs::EntityType cm::add_entity() {
  static EntityType counter = 0;
  const auto ret = counter++;
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
    auto &m = get_map<gl_object>();
    m.erase(idx);
  } break;
  case ct::PARAMETRIC_COM: {
    auto &m = get_map<parametric>();
    m.erase(idx);
  } break;
  case ct::TAG_CURSOR: {
    auto &m = get_map<cursor_params>();
    m.erase(idx);
  } break;
  case ct::TAG_FIGURE: {
    auto &m = get_map<tag_figure>();
    m.erase(idx);
  } break;
  case ct::TAG_POINT: {
    auto &m = get_map<tag_point>();
    m.erase(idx);
  } break;
  case ct::TORUS_COM: {
    auto &m = get_map<torus_params>();
    m.erase(idx);
  } break;
  case ct::TRANSFORMATION_COM: {
    auto &m = get_map<transformation>();
    m.erase(idx);
  } break;

  default:
    return;
  }
}

} // namespace ecs
