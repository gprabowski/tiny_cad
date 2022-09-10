#include "ecs.h"
#include <constructors.h>
#include <registry.h>
#include <shader_manager.h>
#include <systems.h>

#include <log.h>

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

  auto wrap = [](auto v) {while(v < 0.f) { v += 1.0f; } while(v > 1.f) {v -= 1.0f;} return v;};

  ecs::EntityType ret{};

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

  // get even closer with gradients
  float dist = 1.f;
  auto iter = 0;
  while(dist > 1e-3 && iter < 5000) {
    ++iter;
    auto P = first.sample(su, sv);
    auto Pu = first.der_u(su, sv);
    auto Pv = first.der_v(su, sv);
    auto Q = second.sample(ss, st);
    auto Qs = second.der_u(ss, st);
    auto Qt = second.der_v(ss, st);

    dist = glm::length(P - Q);
    TINY_CAD_INFO("INTERSECTION_LOG: dist : {0}\n", dist);
    glm::vec4 grad = glm::length(P - Q) * glm::vec4{
      2 * P.x * Pu.x - 2 * Pu.x * Q.x + 2 * Pu.y * P.y - 2 * Pu.y * Q.y + 2 * Pu.z * P.z - 2 * Pu.z * Q.z,
      2 * P.x * Pv.x - 2 * Pv.x * Q.x + 2 * Pv.y * P.y - 2 * Pv.y * Q.y + 2 * Pv.z * P.z - 2 * Pv.z * Q.z,
      2 * Q.x * Qs.x - 2 * Qs.x * P.x + 2 * Qs.y * Q.y - 2 * Qs.y * P.y + 2 * Qs.z * Q.z - 2 * Qs.z * P.z,
      2 * Q.x * Qt.x - 2 * Qt.x * P.x + 2 * Qt.y * Q.y - 2 * Qt.y * P.y + 2 * Qt.z * Q.z - 2 * Qt.z * P.z
    };

    float delta = std::max(1e-8f, 1e-4f / float(std::pow(10, iter / 500)));
    su -= delta * grad.x;
    sv -= delta * grad.y;
    ss -= delta * grad.z;
    st -= delta * grad.w;

    auto cap = [](auto v) {
      v = std::max(v, 1.0f);
      v = std::min(v, 0.0f);
      return v;
    };

    if(first.u_wrapped) { su = wrap(su); } else { cap(su); }
    if(first.v_wrapped) { sv = wrap(sv); } else { cap(sv); }
    if(second.u_wrapped) { ss = wrap(ss); } else { cap(ss); }
    if(second.v_wrapped) { st = wrap(st); } else { cap(st); }

  }
  TINY_CAD_INFO("INTERSECTION_LOG: dist : {0}\n", dist);

  std::vector<glm::vec3> points {first.sample(su, sv), second.sample(ss, st)}; 

  auto pnormal = [&](auto u, auto v) {return glm::normalize(glm::cross(first.der_u(u, v), first.der_v(u, v))); };
  auto qnormal = [&](auto s, auto t) {return glm::normalize(glm::cross(second.der_u(s, t), second.der_v(s, t))); };
  auto get_tangent = [&](auto u, auto v, auto s, auto t) {return glm::normalize(glm::cross(pnormal(u, v), qnormal(s, t)));};

  constexpr float d = 0.01f;

  glm::vec4 last_coords = {su, sv, ss, st};


  while(true) {
    auto P0 = first.sample(last_coords.x, last_coords.y);

    glm::vec4 next_coords = last_coords;
    auto tangent = get_tangent(last_coords.x, last_coords.y, last_coords.z, last_coords.w);

    // newton away baby
    for(int i = 0; i < 300; ++i) {
      auto get_f = [&](auto u, auto v, auto s, auto t) {
        const auto p1 = first.sample(u, v);
        const auto q1 = second.sample(s, t);
        return glm::vec4{p1 - q1, glm::dot(p1 - P0, tangent) - d};
      };

      auto get_j = [&](auto u, auto v, auto s, auto t) {
        const auto h = 1e-3f;
        const auto v_o = get_f(u, v, s, t);
        auto v1 = (get_f(u + h, v, s, t) - v_o) * (1.f / h);
        auto v2 = (get_f(u, v + h, s, t) - v_o) * (1.f / h);
        auto v3 = (get_f(u, v, s + h, t) - v_o) * (1.f / h);
        auto v4 = (get_f(u, v, s, t + h) - v_o) * (1.f / h);
        return glm::inverse(glm::mat4(v1, v2, v3, v4));
      };

      auto get_newton_decrement = [&](auto u, auto v, auto s, auto t) {
        auto f = get_f(u, v, s, t);
        auto inv_j_f = get_j(u, v, s, t);
        return inv_j_f * f; 
      };

      next_coords -= get_newton_decrement(next_coords.x, next_coords.y, next_coords.z, next_coords.w);

      if(first.u_wrapped) next_coords.x = wrap(next_coords.x);
      if(first.v_wrapped) next_coords.y = wrap(next_coords.y);
      if(second.u_wrapped) next_coords.z = wrap(next_coords.z);
      if(second.v_wrapped) next_coords.w = wrap(next_coords.w);
    }

    points.push_back(first.sample(next_coords.x, next_coords.y));
    points.push_back(second.sample(next_coords.z, next_coords.w));

    last_coords = next_coords;
    if(points.size() > 2000) {
      break;
    }
  }

  for(auto& poi : points) {
    auto pos = poi;
    transformation t;
    t.translation = pos;
    ret = constructors::add_point(
        std::move(t), sm.programs[shader_t::POINT_SHADER].idx);
    auto &gl = reg.get_component<gl_object>(ret);
    gl.color = gl.primary = gl.selected = {0.0f, 1.0f, 0.0f, 1.0f};
  }

  return ret;
}

}
