#include "ecs.h"
#include <constructors.h>
#include <registry.h>
#include <shader_manager.h>
#include <systems.h>

namespace systems {

constexpr int subd{4};

// general case solver
template <typename P_t, typename Q_t>
ecs::EntityType find_intersection(P_t Pfunc, Q_t Qfunc) {
  const auto tmp1 = Pfunc(0.0, 0.0);
  const auto tmp2 = Qfunc(0.0, 0.0);

  if (glm::distance(tmp1, tmp2) < 1.0f) {
    return ecs::null_entity;
  } else {
    return ecs::null_entity;
  }
}

struct res_tmp {
  int i, j, k, l;
  float dist;
  float u, v, s, t;
};

res_tmp get_part(float su1, float sv1, float su2, float sv2, float width,
                 sampler &s1, sampler &s2) {
  // 1. divide into 16 parts the first one
  res_tmp res;
  float granularity = width / subd;

  float _res = glm::length(s1.sample(su1, sv1) - s2.sample(su2, sv2));
  res = {0, 0, 0, 0, _res, su1, sv1, su2, sv2};

  for (int i = 0; i < subd; ++i) {
    for (int j = 0; j < subd; ++j) {
      for (int k = 0; k < subd; ++k) {
        for (int l = 0; l < subd; ++l) {
          float u_1 = su1 + granularity * i + granularity * 0.5f;
          float v_1 = sv1 + granularity * j + granularity * 0.5f;

          float u_2 = su2 + granularity * k + granularity * 0.5f;
          float v_2 = sv2 + granularity * l + granularity * 0.5f;

          auto tmp_res = glm::length(s1.sample(u_1, v_1) - s2.sample(u_2, v_2));
          if (tmp_res < res.dist) {
            res = {i, j, k, l, tmp_res, u_1, v_1, u_2, v_2};
          }
        }
      }
    }
  }

  return res;
}

ecs::EntityType intersect(sampler &first, sampler &second) {
  auto &sm = shader_manager::get_manager();
  auto &reg = ecs::registry::get_registry();

  float su{0.0f}, sv{0.0f}, ss{0.0f}, st{0.0f};
  float width = 1.0f;
  // run loop until you find starting point
  res_tmp r;
  for (int rec = 0; rec < 10; ++rec) {
    r = get_part(su, sv, ss, st, width, first, second);
    width = width / subd;
    su += r.i * width;
    sv += r.j * width;

    ss += r.k * width;
    st += r.l * width;
  }
  auto pos = first.sample(r.u, r.v);
  transformation t;
  t.translation = pos;
  const auto ret = constructors::add_point(
      std::move(t), sm.programs[shader_t::POINT_SHADER].idx);
  auto &gl = reg.get_component<gl_object>(ret);
  gl.color = gl.primary = gl.selected = {0.0f, 1.0f, 0.0f, 1.0f};

  pos = second.sample(r.s, r.t);
  t.translation = pos;
  const auto ret2 = constructors::add_point(
      std::move(t), sm.programs[shader_t::POINT_SHADER].idx);
  auto &gl2 = reg.get_component<gl_object>(ret2);
  gl2.color = gl2.primary = gl2.selected = {1.0f, 0.0f, 0.0f, 1.0f};

  return ret;
}

}
