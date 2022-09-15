#pragma once

#include <algorithm>
#include <cstdint>
#include <map>
#include <stdexcept>
#include <type_traits>
#include <unordered_map>
#include <vector>

#include <Serializer.h>
#include <bezier_surface.h>
#include <bspline_surface.h>
#include <cursor_params.h>
#include <curves.h>
#include <ecs.h>
#include <figure.h>
#include <gl_object.h>
#include <gregory.h>
#include <parametric.h>
#include <relationship.h>
#include <tags.h>
#include <torus.h>
#include <transformation.h>

namespace ecs {

constexpr int num_components = 24;

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
template <> struct com_id<bezierc> : com_id_impl<8> {};
template <> struct com_id<relationship> : com_id_impl<9> {};
template <> struct com_id<tag_parent> : com_id_impl<10> {};
template <> struct com_id<bspline> : com_id_impl<11> {};
template <> struct com_id<tag_virtual> : com_id_impl<12> {};
template <> struct com_id<tag_visible> : com_id_impl<13> {};
template <> struct com_id<tag_clickable> : com_id_impl<14> {};
template <> struct com_id<tag_center_of_weight> : com_id_impl<15> {};
template <> struct com_id<icurve> : com_id_impl<16> {};
template <> struct com_id<tag_bezier_surface_builder> : com_id_impl<17> {};
template <> struct com_id<bezier_surface_params> : com_id_impl<18> {};
template <> struct com_id<tag_bspline_surface_builder> : com_id_impl<19> {};
template <> struct com_id<bspline_surface_params> : com_id_impl<20> {};
template <> struct com_id<tag_gregory> : com_id_impl<21> {};
template <> struct com_id<tags_selection_rect> : com_id_impl<22> {};
template <> struct com_id<tag_intersection> : com_id_impl<23> {};

template <typename T> constexpr component_bitset get_com_bit() {
  return 1ull << com_id<T>::value;
};

template <int ID> struct type_from_id {
  using type =
      typename select<ID, parametric, transformation, gl_object, torus_params,
                      tag_figure, cursor_params, tag_point, selected, bezierc,
                      relationship, tag_parent, bspline, tag_virtual,
                      tag_visible, tag_clickable, tag_center_of_weight, icurve,
                      tag_bezier_surface_builder, bezier_surface_params,
                      tag_bspline_surface_builder, bspline_surface_params,
                      tag_gregory, tags_selection_rect, tag_intersection>::type;
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
                  component_owner<torus_params>,
                  component_owner<tag_figure>,
                  component_owner<tag_point>,
                  component_owner<bezierc>,
                  component_owner<cursor_params>,
                  component_owner<selected>,
                  component_owner<relationship>,
                  component_owner<tag_parent>,
                  component_owner<bspline>,
                  component_owner<tag_virtual>,
                  component_owner<tag_visible>,
                  component_owner<tag_clickable>,
                  component_owner<tag_center_of_weight>,
                  component_owner<icurve>,
                  component_owner<tag_bezier_surface_builder>,
                  component_owner<bezier_surface_params>,
                  component_owner<tag_bspline_surface_builder>,
                  component_owner<bspline_surface_params>,
                  component_owner<tag_gregory>,
                  component_owner<tags_selection_rect>,
                  component_owner<tag_intersection> {

  std::unordered_map<EntityType, component_bitset> entities;

  EntityType counter{0};

  void reset();
  EntityType add_entity();
  void delete_entity(EntityType idx);
  void load_from_scene(const MG1::Scene &scene);
  void get_scene(MG1::Scene &scene);

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

  template <typename First> void reset_component() {
    auto &m = get_map<First>();
    m.clear();
  }

  template <typename First, typename... T> void reset_all_components() {
    reset_component<First>();
    reset_all_components<T...>();
  }

  template <component_bitset N = num_components - 1>
  std::enable_if_t<N != 0, void> reset_all_components() {
    reset_component<typename type_from_id<N>::type>();
    reset_all_components<N - 1>();
  }

  template <component_bitset N = num_components - 1>
  std::enable_if_t<N == 0, void> reset_all_components() {
    reset_component<typename type_from_id<0>::type>();
  }

  template <typename T>
  std::enable_if_t<
      !std::is_same_v<T, relationship> && !std::is_same_v<T, selected> &&
          !std::is_same_v<T, bezierc> && !std::is_same_v<T, bspline> &&
          !std::is_same_v<T, bezier_surface_params> &&
          !std::is_same_v<T, bspline_surface_params>,
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
  std::enable_if_t<std::is_same_v<T, bezierc> || std::is_same_v<T, bspline>,
                   void>
  remove_component(EntityType idx) {
    const auto s = get_component<T>(idx);
    delete_entity(s.bezier_polygon);
    if constexpr (std::is_same_v<T, bspline>) {
      delete_entity(s.deboor_polygon);
    }
    get_map<T>().erase(idx);
    entities[idx] &= ~get_com_bit<T>();
  }

  template <typename T>
  std::enable_if_t<std::is_same_v<T, bezier_surface_params>, void>
  remove_component(EntityType idx) {
    const auto s = get_component<T>(idx);
    if (exists(s.bezier_polygon)) {
      delete_entity(s.bezier_polygon);
    }
    get_map<T>().erase(idx);
    entities[idx] &= ~get_com_bit<T>();
  }

  template <typename T>
  std::enable_if_t<std::is_same_v<T, bspline_surface_params>, void>
  remove_component(EntityType idx) {
    const auto s = get_component<T>(idx);
    if (exists(s.deboor_polygon)) {
      delete_entity(s.deboor_polygon);
    }
    get_map<T>().erase(idx);
    entities[idx] &= ~get_com_bit<T>();
  }

  template <typename T>
  std::enable_if_t<std::is_same_v<T, relationship>, void>
  remove_component(EntityType id) {
    auto &rel = get_component<relationship>(id);
    if (rel.indestructible_counter > 0)
      return;
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
        crel.indestructible_counter -= rel.indestructible_relation;
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
