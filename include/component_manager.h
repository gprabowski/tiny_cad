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
#include <type_traits>

namespace ecs {

constexpr int num_components = 13;

template <typename C> using ComponentStorage = std::map<EntityType, C>;

template <unsigned int N, typename... Cases> struct select;

template <unsigned int N, typename T, typename... Cases>
struct select<N, T, Cases...> : select<N - 1, Cases...> {};

template <typename T, typename... Cases> struct select<0u, T, Cases...> {
  typedef T type;
};

struct null_component;

template <unsigned int ID> struct com_id_impl {
  static constexpr unsigned int value = ID;
};

template <typename T> struct com_id : com_id_impl<63> {};

template <> struct com_id<parametric> : com_id_impl<0> {};
template <> struct com_id<transformation> : com_id_impl<1> {};
template <> struct com_id<gl_object> : com_id_impl<2> {};
template <> struct com_id<torus_params> : com_id_impl<3> {};
template <> struct com_id<tag_figure> : com_id_impl<4> {};
template <> struct com_id<cursor_params> : com_id_impl<5> {};
template <> struct com_id<tag_point> : com_id_impl<6> {};
template <> struct com_id<selected> : com_id_impl<7> {};
template <> struct com_id<tag_bezierc> : com_id_impl<8> {};
template <> struct com_id<relationship> : com_id_impl<9> {};
template <> struct com_id<tag_parent> : com_id_impl<10> {};
template <> struct com_id<secondary_object> : com_id_impl<11> {};
template <> struct com_id<adaptive> : com_id_impl<12> {};

template <typename T> constexpr component_bitset get_com_bit() {
  return 1ull << com_id<T>::value;
};

template <int ID> struct type_from_id {
  using type = typename select<ID, parametric, transformation, gl_object,
                               torus_params, tag_figure, cursor_params,
                               tag_point, selected, tag_bezierc, relationship,
                               tag_parent, secondary_object, adaptive>::type;
};

template <int ID> using type_from_id_t = typename type_from_id<ID>::type;

template <typename T> struct component_owner {
  ComponentStorage<T> component;
  template <typename C = T> ComponentStorage<T> &get_component() {
    return component;
  }
};

struct component_manager {
  std::unordered_map<EntityType, component_bitset> entities;

  std::array<void *, num_components> components;
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
    return get_com_bit<C>() != get_com_bit<null_component>();
  }

  template <typename First> void remove_components(EntityType idx) {
    remove_component<First>(idx);
  }

  template <typename First, typename... T>
  void remove_components(EntityType idx) {
    remove_component<First>(idx);
    remove_components<T...>(idx);
  }

  template <component_bitset N = num_components - 1>
  std::enable_if_t<N != 0, void> remove_all_components(EntityType idx) {
    if (has_component<typename type_from_id<N>::type>(idx)) {
      remove_component<typename type_from_id<N>::type>(idx);
    }
    remove_all_components<N - 1>(idx);
  }

  template <component_bitset N = num_components - 1>
  std::enable_if_t<N == 0, void> remove_all_components(EntityType idx) {
    if (has_component<typename type_from_id<0>::type>(idx)) {
      remove_component<typename type_from_id<0>::type>(idx);
    }
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
        if (!has_component<relationship>(c)) {
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

private:
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
