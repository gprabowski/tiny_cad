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

constexpr int num_components = 18;

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
template <> struct com_id<tag_bspline> : com_id_impl<13> {};
template <> struct com_id<tag_virtual> : com_id_impl<14> {};
template <> struct com_id<tag_visible> : com_id_impl<15> {};
template <> struct com_id<tag_clickable> : com_id_impl<16> {};
template <> struct com_id<tag_center_of_weight> : com_id_impl<17> {};

template <typename T> constexpr component_bitset get_com_bit() {
  return 1ull << com_id<T>::value;
};

template <int ID> struct type_from_id {
  using type =
      typename select<ID, parametric, transformation, gl_object, torus_params,
                      tag_figure, cursor_params, tag_point, selected,
                      tag_bezierc, relationship, tag_parent, secondary_object,
                      adaptive, tag_bspline, tag_virtual, tag_visible,
                      tag_clickable, tag_center_of_weight>::type;
};

template <int ID> using type_from_id_t = typename type_from_id<ID>::type;

template <typename T> struct component_owner {
  ComponentStorage<T> component;
  template <typename C = T> ComponentStorage<T> &get_component() {
    return component;
  }
};

struct registry : component_owner<parametric>,
                  component_owner<transformation>,
                  component_owner<gl_object>,
                  component_owner<secondary_object>,
                  component_owner<torus_params>,
                  component_owner<tag_figure>,
                  component_owner<tag_point>,
                  component_owner<tag_bezierc>,
                  component_owner<cursor_params>,
                  component_owner<selected>,
                  component_owner<relationship>,
                  component_owner<tag_parent>,
                  component_owner<adaptive>,
                  component_owner<tag_bspline>,
                  component_owner<tag_virtual>,
                  component_owner<tag_visible>,
                  component_owner<tag_clickable>,
                  component_owner<tag_center_of_weight> {

  std::unordered_map<EntityType, component_bitset> entities;

  EntityType add_entity();
  void delete_entity(EntityType idx);

  template <typename C>
  std::enable_if_t<!std::is_same_v<C, selected>, bool>
  add_component(EntityType e, C &&c) {
    if (!exists(e) || !component_exists<C>() || has_component<C>(e)) {
      return false;
    }

    auto &reg = get_map<C>();

    reg[e] = std::move(c);
    entities[e] |= get_com_bit<C>();

    return true;
  }

  template <typename C>
  std::enable_if_t<std::is_same_v<C, selected>, bool>
  add_component(EntityType e, selected &&c) {
    if (!exists(e) || !component_exists<C>() || has_component<C>(e)) {
      return false;
    }

    auto &reg = get_map<C>();

    reg[e] = std::move(c);
    entities[e] |= get_com_bit<C>();

    if (has_component<gl_object>(e)) {
      auto &g = get_component<gl_object>(e);
      g.color = g.selected;
    }

    return true;
  }

  template <typename C> C &get_component(EntityType e) {
    if (!exists(e) || !component_exists<C>() || !has_component<C>(e)) {
      throw;
    }

    auto &reg = get_map<C>();

    return reg[e];
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
  std::enable_if_t<!std::is_same_v<T, selected>, void>
  clear_component() {
    auto &m = get_map<T>();
    for(const auto&[idx, _] : m) {
       entities[idx] &= ~get_com_bit<T>();
    }
    m.clear();
  }

  template <typename T>
  std::enable_if_t<std::is_same_v<T, selected>, void>
  clear_component() {
    auto &m = get_map<T>();
    for(const auto&[idx, _] : m) {
       entities[idx] &= ~get_com_bit<T>();
       auto& g = get_component<gl_object>(idx);
       g.color = g.primary;
    }
    m.clear();
  }

  template <typename T>
  std::enable_if_t<!std::is_same_v<T, relationship> &&
                       !std::is_same_v<T, secondary_object> &&
                       !std::is_same_v<T, selected>,
                   void>
  remove_component(EntityType idx) {
    auto &m = get_map<T>();
    m.erase(idx);
    entities[idx] &= ~get_com_bit<T>();
  }

  template <typename T>
  std::enable_if_t<std::is_same_v<T, selected>, void>
  remove_component(EntityType idx) {
    if (has_component<gl_object>(idx)) {
      auto &g = get_component<gl_object>(idx);
      g.color = g.primary;
    }
    auto &m = get_map<T>();
    m.erase(idx);
    entities[idx] &= ~get_com_bit<T>();
  }

  template <typename T>
  std::enable_if_t<std::is_same_v<T, secondary_object>, void>
  remove_component(EntityType idx) {
    const auto s = get_component<secondary_object>(idx);
    delete_entity(s.val);
    get_map<secondary_object>().erase(idx);
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
          get_map<relationship>().erase(c);
          entities[c] &= ~get_com_bit<relationship>();
        }
      }
    }
    get_map<relationship>().erase(id);
    entities[id] &= ~get_com_bit<relationship>();
  }

  template <typename T>
  std::enable_if_t<!std::is_same_v<T, selected>, void> remove_all() {
    auto &m = get_map<T>();
    for (auto &[idx, c] : m) {
      entities[idx] &= ~get_com_bit<T>();
    }
    m.clear();
  }

  template <typename T>
  std::enable_if_t<std::is_same_v<T, selected>, void> remove_all() {
    auto &m = get_map<T>();
    for (auto &[idx, c] : m) {
      if (has_component<gl_object>(idx)) {
        auto &g = get_component<gl_object>(idx);
        g.color = g.primary;
      }
      entities[idx] &= ~get_com_bit<T>();
    }
    m.clear();
  }

  template <typename T> constexpr ComponentStorage<T> &get_map() {
    return static_cast<component_owner<T> *>(this)->get_component();
  }

  static registry &get_registry() {
    static registry reg;
    return reg;
  }

  registry &operator=(const registry &r) = delete;
  registry &operator=(const registry &&r) = delete;
  registry(const registry &) = delete;
  registry(const registry &&) = delete;

private:
  registry(){};
};

} // namespace ecs
