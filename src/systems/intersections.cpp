#include "ecs.h"
#include <constructors.h>
#include <registry.h>
#include <shader_manager.h>
#include <systems.h>

#include <log.h>

namespace systems {

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

inline float wrap(float v) {
  while (v < 0.f) {
    v += 1.0f;
  }
  while (v > 1.f) {
    v -= 1.0f;
  }
  return v;
};

res_tmp get_part(int subd, float su1, float sv1, float su2, float sv2, float width,
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

void find_subdivision_start_point(int subd, float& su, float& sv, float& ss, float& st, sampler& first, sampler& second) {
  float width = 1.0f;
  // run loop until you find starting point
  res_tmp r;
  for (int rec = 0; rec < 10; ++rec) {
    r = get_part(subd, su, sv, ss, st, width, first, second);
    width = width / subd;

    su += r.i * width;
    sv += r.j * width;
    ss += r.k * width;
    st += r.l * width;
  }
}

intersection_status improve_start_point_with_gradients(float& su, float& sv, float& ss, float& st, 
                                        float start_acceptance, int gradient_iters, float start_delta,
                                        sampler& first, sampler& second) {
  float dist = glm::length(first.sample(su, sv) - second.sample(ss, st));
  auto iter = 0;
  while (dist > start_acceptance && iter < gradient_iters) {
    ++iter;
    auto P = first.sample(su, sv);
    auto Pu = first.der_u_opt(su, sv, P);
    auto Pv = first.der_v_opt(su, sv, P);
    auto Q = second.sample(ss, st);
    auto Qs = second.der_u_opt(ss, st, Q);
    auto Qt = second.der_v_opt(ss, st, Q);

    dist = glm::length(P - Q);
    glm::vec4 grad =
        glm::length(P - Q) *
        glm::vec4{2 * P.x * Pu.x - 2 * Pu.x * Q.x + 2 * Pu.y * P.y -
                      2 * Pu.y * Q.y + 2 * Pu.z * P.z - 2 * Pu.z * Q.z,
                  2 * P.x * Pv.x - 2 * Pv.x * Q.x + 2 * Pv.y * P.y -
                      2 * Pv.y * Q.y + 2 * Pv.z * P.z - 2 * Pv.z * Q.z,
                  2 * Q.x * Qs.x - 2 * Qs.x * P.x + 2 * Qs.y * Q.y -
                      2 * Qs.y * P.y + 2 * Qs.z * Q.z - 2 * Qs.z * P.z,
                  2 * Q.x * Qt.x - 2 * Qt.x * P.x + 2 * Qt.y * Q.y -
                      2 * Qt.y * P.y + 2 * Qt.z * Q.z - 2 * Qt.z * P.z};

    float delta = std::max(1e-10f, start_delta / float(std::pow(10, iter / 500)));
    su -= delta * grad.x;
    sv -= delta * grad.y;
    ss -= delta * grad.z;
    st -= delta * grad.w;

    if (first.u_wrapped) {
      su = wrap(su);
    } else if(su > 1.0f || su < 0.0f){
      return intersection_status::start_point_gradient_edge_error;
    }
    if (first.v_wrapped) {
      sv = wrap(sv);
    } else if(sv > 1.0f || sv < 0.0f){
      return intersection_status::start_point_gradient_edge_error;
    }
    if (second.u_wrapped) {
      ss = wrap(ss);
    } else if(ss > 1.0f || ss < 0.0f){
      return intersection_status::start_point_gradient_edge_error;
    }
    if (second.v_wrapped) {
      st = wrap(st);
    } else if(st > 1.0f || st < 0.0f){
      return intersection_status::start_point_gradient_edge_error;
    }
  } 
  return intersection_status::success;
}

intersection_status find_start_point(const intersection_params& params, float& su, float& sv, 
                                     float& ss, float&st, sampler& first, sampler& second) {
  find_subdivision_start_point(params.subdivisions, su, sv, ss, st, first, second);

  float dist = glm::length(first.sample(su, sv) - second.sample(ss, st));
  if(dist > params.subdivisions_acceptance) {
    return intersection_status::start_point_subdivisions_final_point_error;
  }

  // get even closer with gradients
  auto gradient_status = improve_start_point_with_gradients(su, sv, ss, st, params.start_acceptance, params.gradient_iters,
                                                            params.start_delta, first, second);

  dist = glm::length(first.sample(su, sv) - second.sample(ss, st));

  if(dist > params.start_acceptance) {
    return intersection_status::start_point_gradient_final_point_error;
  }

  return gradient_status;
}


// vector making sure that two points are close,
// in distance d from previous point and on a tangent plane
// precomputed version
inline glm::vec4 get_f_opt(const glm::vec3& p1, const glm::vec3& q1, 
                    const glm::vec3& P0, const glm::vec3& tangent, 
                    float delta) {
  return glm::vec4{p1 - q1, glm::dot(p1 - P0, tangent) - delta};
};

inline glm::mat4 get_j (const glm::vec3& p_def, const glm::vec3& q_def, 
            float u, float v, float s, float t,
            const glm::vec4& v_o, const glm::vec3& P0,
            const glm::vec3& tangent, sampler& first, 
            sampler& second, const float delta) {
  constexpr auto h = 1e-3f;
  const auto v1 =
      (get_f_opt(first.sample(u + h, v), q_def, P0, tangent, delta) - v_o) * (1.f / h);
  const auto v2 =
      (get_f_opt(first.sample(u, v + h), q_def, P0, tangent, delta) - v_o) * (1.f / h);
  const auto v3 =
      (get_f_opt(p_def, second.sample(s + h, t), P0, tangent, delta) - v_o) * (1.f / h);
  const auto v4 =
      (get_f_opt(p_def, second.sample(s, t + h), P0, tangent, delta) - v_o) * (1.f / h);
  return glm::inverse(glm::mat4(v1, v2, v3, v4));
};

auto get_newton_decrement(float u, float v, float s, float t, 
                          float delta, sampler& first, sampler& second,
                          const glm::vec3& tangent, const glm::vec3& P0, 
                          const glm::vec3& p_def, const glm::vec3& q_def) {
  auto f = get_f_opt(p_def, q_def, P0, tangent, delta);
  auto inv_j_f = get_j(p_def, q_def, u, v, s, t, f, P0, tangent, first, second, delta);
  return inv_j_f * f;
};

inline glm::vec3 get_tangent(float u, float v, float s, float t, sampler& first, sampler& second) {
  auto pnormal = [&](auto u, auto v) {
    return glm::normalize(glm::cross(first.der_u(u, v), first.der_v(u, v)));
  };

  auto qnormal = [&](auto s, auto t) {
    return glm::normalize(glm::cross(second.der_u(s, t), second.der_v(s, t)));
  };

  return glm::normalize(glm::cross(pnormal(u, v), qnormal(s, t)));
}

inline void extend_start_point(std::vector<glm::vec3>& points, const glm::vec4& start_coords, 
                               float delta, sampler& first, sampler& second,
                               const intersection_params& params) {
  auto first_point = points[0];
  auto last_coords = start_coords;
  auto iter = 0;
  while (iter < 10000) {
    ++iter;
    auto P0 = first.sample(last_coords.x, last_coords.y);
    glm::vec4 next_coords = last_coords;
    const auto tangent =
        get_tangent(last_coords.x, last_coords.y, last_coords.z, last_coords.w, first, second);

    // newton away baby
    auto q_def = second.sample(next_coords.z, next_coords.w);
    auto p_def = first.sample(next_coords.x, next_coords.y);
    for (int i = 0; i < params.newton_iters; ++i) {
      next_coords -= get_newton_decrement(next_coords.x, next_coords.y,
                                          next_coords.z, next_coords.w,
                                          delta, first, second, tangent, P0, p_def, q_def);
      if (first.u_wrapped) {
        next_coords.x = wrap(next_coords.x);
      }
      if (first.v_wrapped) {
        next_coords.y = wrap(next_coords.y);
      }
      if (second.u_wrapped) {
        next_coords.z = wrap(next_coords.z);
      }
      if (second.v_wrapped) {
        next_coords.w = wrap(next_coords.w);
      }
      if(glm::length(p_def - q_def) < params.newton_acceptance) {
        break; 
      }
    }

    auto last_point = points[points.size() - 1];

    // check if cycle 
    if(glm::length(first_point - last_point) < std::abs(delta) && points.size() > 4) {
      points.push_back(first_point);
      break;
    }

    // Check if Newton is converging
    if((next_coords.x > 1.0f || next_coords.x < 0.0f) ||
        (next_coords.y > 1.0f || next_coords.y < 0.0f) ||
        (next_coords.z > 1.0f || next_coords.z < 0.0f) ||
        (next_coords.w > 1.0f || next_coords.w < 0.0f) ||
        (glm::length(first.sample(next_coords.x, next_coords.y) - second.sample(next_coords.z, next_coords.w)) > std::abs(delta))) {
      break;
    }
    points.push_back(first.sample(next_coords.x, next_coords.y));
    last_coords = next_coords;
  }
}

intersection_status find_all_intersection_points(const intersection_params& params, glm::vec4 start_coords,
                                                 std::vector<glm::vec3>& points, sampler& first, sampler& second) {
  auto first_point = points[0];

  // find first line of points (or cycle)
  extend_start_point(points, start_coords, params.delta, first, second, params);
  auto last_point = points[points.size() - 1];

  const bool cycle_found = (glm::length(first_point - last_point) < std::abs(params.delta) && points.size() > 4);

  // no cycle, we need to do reverse delta now
  std::vector<glm::vec3> tmp_points {first_point};
  if(!cycle_found) {
    extend_start_point(tmp_points, start_coords, -params.delta, first, second, params);
  }

  // combine two parts
  points.insert(points.begin(), tmp_points.rbegin(), tmp_points.rend());
  return intersection_status::success;
}


intersection_status intersect(sampler &first, sampler &second, const intersection_params& params) {
  auto &sm = shader_manager::get_manager();
  auto &reg = ecs::registry::get_registry();
  
  float su{0.0f}, sv{0.0f}, ss{0.0f}, st{0.0f};

  // find start point
  auto start_point_status = find_start_point(params, su, sv, ss, st, first, second);
  if(start_point_status != intersection_status::success) {
    return start_point_status;
  }

  // add start point
  std::vector<glm::vec3> points{first.sample(su, sv), second.sample(ss, st)};
  glm::vec4 start_coords = {su, sv, ss, st};

  // find the entire intersection
  auto newton_status = find_all_intersection_points(params, start_coords, points, first, second);

  if(newton_status != intersection_status::success) {
    return newton_status;
  }

  for (auto &poi : points) {
    auto pos = poi;
    transformation t;
    t.translation = pos;
    auto ret = constructors::add_point(std::move(t),
                                  sm.programs[shader_t::POINT_SHADER].idx);
    auto &gl = reg.get_component<gl_object>(ret);
    gl.color = gl.primary = gl.selected = {0.0f, 1.0f, 0.0f, 1.0f};
  }

  return intersection_status::success;
}
} // namespace systems
