#pragma once

#include "torus.h"
#include <cstdint>
#include <map>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include <cursor_params.h>
#include <ecs.h>
#include <gl_object.h>
#include <parametric.h>
#include <tags.h>
#include <transformation.h>

namespace ecs {

struct component_manager {
  std::unordered_map<EntityType, component_bitset> entities;

  std::map<EntityType, parametric> parametric_components;
  std::map<EntityType, transformation> transformation_components;
  std::map<EntityType, gl_object> ogl_components;
  std::map<EntityType, torus_params> torus_components;
  std::map<EntityType, tag_figure> figure_component;
  std::map<EntityType, tag_point> point_component;
  std::map<EntityType, cursor_params> cursor_component;

  EntityType add_entity();
  void delete_entity(EntityType idx);

  template <typename C> bool add_component(EntityType e, C &&c) {
    if (!exists(e) || !component_exists<C>() || has_component<C>(e)) {
      return false;
    }

    auto &cm = get_map<C>();

    cm[e] = std::move(c);
    entities[e] |= get_com_bit<C>();

    return true;
  }

  template <typename C> C &get_component(EntityType e) {
    if (!exists(e) || !component_exists<C>() || !has_component<C>(e)) {
      throw;
    }

    auto &cm = get_map<C>();

    return cm[e];
  }

  template <typename T> inline bool has_component(EntityType e) const {
    return (entities.at(e) & get_com_bit<T>()) != 0;
  }

  inline bool exists(EntityType e) const { return entities.count(e) > 0; }

  template <typename C> inline constexpr bool component_exists() {
    return get_com_bit<C>() != ct::OTHER;
  }

private:
  template <typename C> constexpr ecs::ct get_com_bit() const {
    if constexpr (std::is_same_v<C, parametric>) {
      return ct::PARAMETRIC_COM;
    } else if constexpr (std::is_same_v<C, transformation>) {
      return ct::TRANSFORMATION_COM;
    } else if constexpr (std::is_same_v<C, gl_object>) {
      return ct::OGL_COM;
    } else if constexpr (std::is_same_v<C, torus_params>) {
      return ct::TORUS_COM;
    } else if constexpr (std::is_same_v<C, tag_figure>) {
      return ct::TAG_FIGURE;
    } else if constexpr (std::is_same_v<C, cursor_params>) {
      return ct::TAG_CURSOR;
    } else if constexpr (std::is_same_v<C, tag_point>) {
      return ct::TAG_POINT;
    }
    return ct::OTHER;
  }

  template <typename T> constexpr std::map<EntityType, T> &get_map() {
    using ret_t = std::map<EntityType, T>;
    ret_t ret;
    if constexpr (std::is_same_v<T, parametric>) {
      return (parametric_components);
    } else if constexpr (std::is_same_v<T, transformation>) {
      return (transformation_components);
    } else if constexpr (std::is_same_v<T, gl_object>) {
      return (ogl_components);
    } else if constexpr (std::is_same_v<T, torus_params>) {
      return (torus_components);
    } else if constexpr (std::is_same_v<T, tag_figure>) {
      return (figure_component);
    } else if constexpr (std::is_same_v<T, tag_point>) {
      return (point_component);
    } else if constexpr (std::is_same_v<T, cursor_params>) {
      return (cursor_component);
    }

    throw std::runtime_error("Couldn't find map for this type");
  }

  void remove_component(EntityType idx, ecs::ct cp);
};

} // namespace ecs
