#pragma once

#include "torus.h"
#include <cstdint>
#include <map>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include <cursor_params.h>
#include <ecs.h>
#include <figure.h>
#include <gl_object.h>
#include <parametric.h>
#include <relationship.h>
#include <tags.h>
#include <transformation.h>

namespace ecs {

template <typename C> using ComponentStorage = std::map<EntityType, C>;

struct component_manager {
  std::unordered_map<EntityType, component_bitset> entities;

  ComponentStorage<parametric> parametric_components;
  ComponentStorage<transformation> transformation_components;
  ComponentStorage<gl_object> ogl_components;
  ComponentStorage<torus_params> torus_components;
  ComponentStorage<tag_figure> figure_component;
  ComponentStorage<tag_point> point_component;
  ComponentStorage<tag_bezierc> bezierc_component;
  ComponentStorage<cursor_params> cursor_component;
  ComponentStorage<selected> selected_component;
  ComponentStorage<relationship> relationship_component;

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
  std::enable_if_t<!std::is_same_v<T, relationship>, void>
  remove_component(EntityType idx) {
    auto &m = get_map<T>();
    m.erase(idx);
    entities[idx] &= ~get_com_bit<T>();
  }

  template <typename T>
  std::enable_if_t<std::is_same_v<T, relationship>, void>
  remove_component(EntityType id) {
    auto &rel = get_component<relationship>(id);
      if (rel.parent != ecs::null_entity) {
      // is a child
        auto &parent = get_component<relationship>(rel.parent);
        --parent.num_children;
        if (rel.next_child != id && rel.prev_child != id) {
          auto &prev = get_component<relationship>(rel.prev_child);
          auto &next = get_component<relationship>(rel.next_child);
          prev.next_child = rel.next_child;
          next.prev_child = rel.prev_child;
        } else if (rel.next_child != id) {
          auto &next = get_component<relationship>(rel.next_child);
          next.prev_child = rel.next_child;
          parent.first_child = rel.next_child;
        } else if (rel.prev_child != id) {
          auto &prev = get_component<relationship>(rel.prev_child);
          prev.next_child = rel.prev_child;
        } else {
          parent.first_child = ecs::null_entity;
        }
      } else if (rel.first_child != null_entity) {
        // parent needs to lose the component and all the kids do too (not
        // recursively)
        auto curr_child = rel.first_child;
        auto next = relationship_component[curr_child].next_child;
        for (std::size_t c = 0; c < rel.num_children; ++c) {
            auto tmp = curr_child;
          curr_child = next;
          next = relationship_component[curr_child].next_child;
          relationship_component.erase(tmp);
          entities[tmp] &= ~get_com_bit<relationship>();
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
    }

    throw std::runtime_error("Couldn't find map for this type");
  }
};

} // namespace ecs
