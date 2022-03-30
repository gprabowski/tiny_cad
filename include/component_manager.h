#pragma once

#include "torus.h"
#include <algorithm>
#include <cstdint>
#include <map>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include <adaptive.h>
#include <cursor_params.h>
#include <ecs.h>
#include <figure.h>
#include <gl_object.h>
#include <parametric.h>
#include <relationship.h>
#include <secondary.h>
#include <tags.h>
#include <transformation.h>

namespace ecs {

template <typename C> using ComponentStorage = std::map<EntityType, C>;

struct component_manager {
  std::unordered_map<EntityType, component_bitset> entities;

  ComponentStorage<parametric> parametric_components;
  ComponentStorage<transformation> transformation_components;
  ComponentStorage<gl_object> ogl_components;
  ComponentStorage<secondary_object> secondary_component;
  ComponentStorage<torus_params> torus_components;
  ComponentStorage<tag_figure> figure_component;
  ComponentStorage<tag_point> point_component;
  ComponentStorage<tag_bezierc> bezierc_component;
  ComponentStorage<cursor_params> cursor_component;
  ComponentStorage<selected> selected_component;
  ComponentStorage<relationship> relationship_component;
  ComponentStorage<tag_parent> parent_component;
  ComponentStorage<adaptive> adaptive_component;

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

  template <typename T>
  std::enable_if_t<!std::is_same_v<T, relationship> &&
                       !std::is_same_v<T, secondary_object>,
                   void>
  remove_component(EntityType idx) {
    auto &m = get_map<T>();
    m.erase(idx);
    entities[idx] &= ~get_com_bit<T>();
  }

  template <typename T>
  std::enable_if_t<std::is_same_v<T, secondary_object>, void>
  remove_component(EntityType idx) {
    const auto s = get_component<secondary_object>(idx);
    delete_entity(s.val);
    secondary_component.erase(idx);
    entities[idx] &= ~get_com_bit<secondary_object>();
  }

  template <typename T>
  std::enable_if_t<std::is_same_v<T, relationship>, void>
  remove_component(EntityType id) {
    auto &rel = get_component<relationship>(id);
    if (rel.parents.size()) {
      for (auto p : rel.parents) {
        // delete from parent this relationship
        auto &prel = get_component<relationship>(p);
        prel.children.erase(std::remove_if(prel.children.begin(),
                                           prel.children.end(),
                                           [=](auto pc) { return pc == id; }),
                            prel.children.end());
      }
    } else if (rel.children.size()) {
      for (auto c : rel.children) {
        if(!has_component<relationship>(c)) {
            continue;
        }
        auto &crel = get_component<relationship>(c);
        crel.parents.erase(std::remove_if(crel.parents.begin(),
                                          crel.parents.end(),
                                          [=](auto cp) { return cp == id; }),
                           crel.parents.end());
        if (0 == crel.parents.size()) {
          relationship_component.erase(c);
          entities[c] &= ~get_com_bit<relationship>();
        }
      }
    }
    relationship_component.erase(id);
    entities[id] &= ~get_com_bit<relationship>();
  }

  template <typename T> void remove_all() {
    auto &m = get_map<T>();
    for (auto &[idx, c] : m) {
      entities[idx] &= ~get_com_bit<T>();
    }
    m.clear();
  }

  void remove_component(EntityType idx, ecs::ct cp);

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
    } else if constexpr (std::is_same_v<C, tag_bezierc>) {
      return ct::TAG_BEZIERC;
    } else if constexpr (std::is_same_v<C, selected>) {
      return ct::TAG_SELECTED;
    } else if constexpr (std::is_same_v<C, relationship>) {
      return ct::RELATIONSHIP;
    } else if constexpr (std::is_same_v<C, tag_parent>) {
      return ct::TAG_PARENT;
    } else if constexpr (std::is_same_v<C, secondary_object>) {
      return ct::SECONDARY;
    } else if constexpr (std::is_same_v<C, adaptive>) {
      return ct::ADAPTIVE;
    }
    return ct::OTHER;
  }

  template <typename T> constexpr ComponentStorage<T> &get_map() {
    using ret_t = ComponentStorage<T>;
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
    } else if constexpr (std::is_same_v<T, tag_bezierc>) {
      return (bezierc_component);
    } else if constexpr (std::is_same_v<T, cursor_params>) {
      return (cursor_component);
    } else if constexpr (std::is_same_v<T, selected>) {
      return (selected_component);
    } else if constexpr (std::is_same_v<T, relationship>) {
      return (relationship_component);
    } else if constexpr (std::is_same_v<T, tag_parent>) {
      return (parent_component);
    } else if constexpr (std::is_same_v<T, secondary_object>) {
      return (secondary_component);
    } else if constexpr (std::is_same_v<T, adaptive>) {
      return (adaptive_component);
    }

    throw std::runtime_error("Couldn't find map for this type");
  }
};

} // namespace ecs
