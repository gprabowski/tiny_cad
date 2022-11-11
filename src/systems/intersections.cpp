#include "ecs.h"
#include <constructors.h>
#include <queue>
#include <random>
#include <registry.h>
#include <shader_manager.h>
#include <systems.h>
#include <utility>

#include <log.h>

namespace systems {

template <int TexSize, typename F>
void Bresenham(float _xa, float _ya, float _xb, float _yb, F putpixel) {
  int xa = TexSize * _xa;
  int ya = TexSize * _ya;
  int xb = TexSize * _xb;
  int yb = TexSize * _yb;

  int dx = abs(xa - xb), dy = abs(ya - yb);
  int p = 2 * dy - dx;
  int twoDy = 2 * dy, twoDyDx = 2 * (dy - dx);
  int x, y, xEnd;
  if (xa > xb) {
    x = xb;
    y = yb;
    xEnd = xa;
  } else {
    x = xa;
    y = ya;
    xEnd = xb;
  }
  putpixel(x, y, 1.f);
  while (x < xEnd) {
    x++;
    if (p < 0) {
      p = p + twoDy;
    } else {
      y++;
      p = p + twoDyDx;
    }
    putpixel(x, y, 1.f);
  }
}

template <int Size>
inline bool is_valid_coord(bool xwrapped, bool ywrapped,
                           std::pair<int, int> &c) {
  if (xwrapped) {
    if (c.first < 0)
      c.first += Size;
    if (c.first >= Size)
      c.first -= Size;
  }

  if (ywrapped) {
    if (c.second < 0)
      c.second += Size;
    if (c.second >= Size)
      c.second -= Size;
  }

  return !(c.first < 0 || c.first >= Size || c.second < 0 || c.second >= Size);
}

template <int TexSize, typename Getter, typename Setter>
void flood_fill(bool xwrapped, bool ywrapped, Getter get, Setter set) {
  int x = 0;
  int y = 0;
  // find starting point
  if (get(0, 0) > 0.5f) {
    for (int i = 0; i < TexSize * TexSize; ++i) {
      const int tmp_x = i % TexSize;
      const int tmp_y = i / TexSize;
      if (get(tmp_x, tmp_y) < 0.5f) {
        x = tmp_x;
        y = tmp_y;
        break;
      }
    }
  }

  std::queue<std::pair<int, int>> coord_queue;
  set(x, y, 1.f);
  coord_queue.push(std::make_pair(x, y));
  while (!coord_queue.empty()) {
    auto [curr_x, curr_y] = coord_queue.front();
    coord_queue.pop();
    auto coord = std::make_pair(curr_x - 1, curr_y);
    if (is_valid_coord<TexSize>(xwrapped, ywrapped, coord) &&
        (get(coord.first, coord.second) < 0.5f)) {
      set(coord.first, coord.second, 1.f);
      coord_queue.push(coord);
    }
    coord = std::make_pair(curr_x + 1, curr_y);
    if (is_valid_coord<TexSize>(xwrapped, ywrapped, coord) &&
        (get(coord.first, coord.second) < 0.5f)) {
      set(coord.first, coord.second, 1.f);
      coord_queue.push(coord);
    }
    coord = std::make_pair(curr_x, curr_y - 1);
    if (is_valid_coord<TexSize>(xwrapped, ywrapped, coord) &&
        (get(coord.first, coord.second) < 0.5f)) {
      set(coord.first, coord.second, 1.f);
      coord_queue.push(coord);
    }
    coord = std::make_pair(curr_x, curr_y + 1);
    if (is_valid_coord<TexSize>(xwrapped, ywrapped, coord) &&
        (get(coord.first, coord.second) < 0.5f)) {
      set(coord.first, coord.second, 1.f);
      coord_queue.push(coord);
    }
  }
}

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

inline bool needs_wrapping(float v) { return v < 0.f || v > 1.f; }

void find_closest_to_cursor(float &u, float &v, float &s, float &t,
                            const glm::vec3 &cursor_pos, sampler &first,
                            sampler &second, bool self_intersection) {
  float smallest_p = glm::length(first.sample(u, v) - cursor_pos);
  float smallest_q = glm::length(second.sample(s, t) - cursor_pos);

  constexpr int iter = 100;
  for (int i = 0; i < iter; ++i) {
    for (int j = 0; j < iter; ++j) {
      float u_1 = i * 1.f / iter;
      float v_1 = j * 1.f / iter;

      auto pos1 = first.sample(u_1, v_1);
      auto pos2 = second.sample(u_1, v_1);

      auto tmp_res_p = glm::length(pos1 - cursor_pos);
      if (!(self_intersection &&
            (std::abs(u_1 - s) < 0.2 && std::abs(v_1 - t) < 0.2))) {
        if (tmp_res_p < smallest_p) {
          u = u_1;
          v = v_1;
          smallest_p = tmp_res_p;
        }
      }

      auto tmp_res_q = glm::length(pos2 - cursor_pos);
      if (!(self_intersection &&
            (std::abs(u_1 - u) < 0.2 && std::abs(v_1 - v) < 0.2))) {
        if (tmp_res_q < smallest_q) {
          s = u_1;
          t = v_1;
          smallest_q = tmp_res_q;
        }
      }
    }
  }

  for (int i = 0; i < iter; ++i) {
    for (int j = 0; j < iter; ++j) {
      float u_1 = i * 1.f / (iter * iter);
      float v_1 = j * 1.f / (iter * iter);

      auto pos1 = first.sample(u + u_1, v + v_1);
      auto pos2 = second.sample(s + u_1, t + v_1);

      auto tmp_res_p = glm::length(pos1 - cursor_pos);
      if (!(self_intersection &&
            (std::abs(u + u_1 - s) < 0.2 && std::abs(v + v_1 - t) < 0.2))) {
        if (tmp_res_p < smallest_p) {
          u += u_1;
          v += v_1;
          smallest_p = tmp_res_p;
        }
      }

      auto tmp_res_q = glm::length(pos2 - cursor_pos);
      if (!(self_intersection &&
            (std::abs(s + u_1 - u) < 0.2 && std::abs(t + v_1 - v) < 0.2))) {
        if (tmp_res_q < smallest_q) {
          s += u_1;
          t += v_1;
          smallest_q = tmp_res_q;
        }
      }
    }
  }
}

res_tmp get_part(int subd, float su1, float sv1, float su2, float sv2,
                 float width, sampler &s1, sampler &s2,
                 bool self_intersection) {
  // 1. divide into 16 parts the first one
  res_tmp res;
  float granularity = width / subd;

  res = {0, 0, 0, 0, 1e10f, su1, sv1, su2, sv2};

  auto is_out_of_bounds = [](auto v) { return v < 0.0f || v > 1.0f; };

  for (int i = 0; i < subd; ++i) {
    for (int j = 0; j < subd; ++j) {
      for (int k = 0; k < subd; ++k) {
        for (int l = 0; l < subd; ++l) {
          float u_1 = su1 + granularity * i + granularity * 0.5f;
          float v_1 = sv1 + granularity * j + granularity * 0.5f;

          float u_2 = su2 + granularity * k + granularity * 0.5f;
          float v_2 = sv2 + granularity * l + granularity * 0.5f;

          if (is_out_of_bounds(u_1))
            continue;
          if (is_out_of_bounds(u_2))
            continue;
          if (is_out_of_bounds(v_1))
            continue;
          if (is_out_of_bounds(v_2))
            continue;

          if (!(self_intersection &&
                (std::abs(u_1 - u_2) < 0.2 && std::abs(v_1 - v_2) < 0.2))) {
            auto pos1 = s1.sample(u_1, v_1);
            auto pos2 = s2.sample(u_2, v_2);
            auto tmp_res = glm::length(pos1 - pos2);
            if (tmp_res < res.dist) {
              res = {i, j, k, l, tmp_res, u_1, v_1, u_2, v_2};
            }
          }
        }
      }
    }
  }
  return res;
}

void find_subdivision_start_point(int subd, int subd_iter, float &su, float &sv,
                                  float &ss, float &st, sampler &first,
                                  sampler &second, bool self_intersection) {
  float width = 1.0f;
  // run loop until you find starting point
  res_tmp r;
  for (int rec = 0; rec < subd_iter; ++rec) {
    r = get_part(subd, su, sv, ss, st, width, first, second, self_intersection);

    width = width / subd;

    su += r.i * width;
    sv += r.j * width;
    ss += r.k * width;
    st += r.l * width;
  }
}

void find_cursor_start_point(int subd, int subd_iter, float &su, float &sv,
                             float &ss, float &st, sampler &first,
                             sampler &second, const glm::vec3 &cursor_pos,
                             float cursor_dist, bool self_intersection) {
  float width = 1.0f;
  // run loop until you find starting point
  find_closest_to_cursor(su, sv, ss, st, cursor_pos, first, second,
                         self_intersection);

  width = 0.05f;
  su -= width / 2;
  sv -= width / 2;
  ss -= width / 2;
  st -= width / 2;
  res_tmp r;
  for (int rec = 0; rec < subd_iter; ++rec) {
    r = get_part(subd, su, sv, ss, st, width, first, second, false);

    width = width / subd;

    su += r.i * width;
    sv += r.j * width;
    ss += r.k * width;
    st += r.l * width;
  }
}

intersection_status improve_start_point_with_gradients(
    float &su, float &sv, float &ss, float &st, float start_acceptance,
    int gradient_iters, float start_delta, sampler &first, sampler &second) {
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

    float delta =
        std::max(1e-10f, start_delta / float(std::pow(10, iter / 500)));
    su -= delta * grad.x;
    sv -= delta * grad.y;
    ss -= delta * grad.z;
    st -= delta * grad.w;

    if (first.u_wrapped) {
      su = wrap(su);
    } else if (su > 1.0f || su < 0.0f) {
      return intersection_status::start_point_gradient_edge_error;
    }
    if (first.v_wrapped) {
      sv = wrap(sv);
    } else if (sv > 1.0f || sv < 0.0f) {
      return intersection_status::start_point_gradient_edge_error;
    }
    if (second.u_wrapped) {
      ss = wrap(ss);
    } else if (ss > 1.0f || ss < 0.0f) {
      return intersection_status::start_point_gradient_edge_error;
    }
    if (second.v_wrapped) {
      st = wrap(st);
    } else if (st > 1.0f || st < 0.0f) {
      return intersection_status::start_point_gradient_edge_error;
    }
  }
  return intersection_status::success;
}

intersection_status find_start_point(const intersection_params &params,
                                     float &su, float &sv, float &ss, float &st,
                                     sampler &first, sampler &second,
                                     bool self_intersection,
                                     const glm::vec3 &cursor_pos) {
  if (params.start_from_cursor) {
    find_cursor_start_point(params.subdivisions, params.subdivisions_iterations,
                            su, sv, ss, st, first, second, cursor_pos,
                            params.cursor_dist, self_intersection);
  } else {
    find_subdivision_start_point(params.subdivisions,
                                 params.subdivisions_iterations, su, sv, ss, st,
                                 first, second, self_intersection);
  }

  auto p_test = first.sample(su, sv);
  auto q_test = second.sample(ss, st);
  float dist = glm::length(p_test - q_test);
  if (dist > params.subdivisions_acceptance) {
    TINY_CAD_INFO("INTERSECTION_LOG: (dist: {6}) ({7} {8})<{0}, {1}, {2}> ({9} "
                  "{10})<{3}, {4}, {5}>",
                  p_test.x, p_test.y, p_test.z, q_test.x, q_test.y, q_test.z,
                  dist, su, sv, ss, st);
    return intersection_status::start_point_subdivisions_final_point_error;
  }

  // get even closer with gradients
  auto gradient_status = improve_start_point_with_gradients(
      su, sv, ss, st, params.start_acceptance, params.gradient_iters,
      params.start_delta, first, second);

  p_test = first.sample(su, sv);
  q_test = second.sample(ss, st);
  dist = glm::length(p_test - q_test);

  if (dist > params.start_acceptance) {
    TINY_CAD_INFO("INTERSECTION_LOG: (dist: {6}) ({7} {8})<{0}, {1}, {2}> ({9} "
                  "{10})<{3}, {4}, {5}>",
                  p_test.x, p_test.y, p_test.z, q_test.x, q_test.y, q_test.z,
                  dist, su, sv, ss, st);
    return intersection_status::start_point_gradient_final_point_error;
  }

  return gradient_status;
}

// vector making sure that two points are close,
// in distance d from previous point and on a tangent plane
// precomputed version
inline glm::vec4 get_f_opt(const glm::vec3 &p1, const glm::vec3 &q1,
                           const glm::vec3 &P0, const glm::vec3 &tangent,
                           float delta) {
  return glm::vec4{p1 - q1, glm::dot(p1 - P0, tangent) - delta};
};

inline glm::mat4 get_j(const glm::vec3 &p_def, const glm::vec3 &q_def, float u,
                       float v, float s, float t, const glm::vec4 &v_o,
                       const glm::vec3 &P0, const glm::vec3 &tangent,
                       sampler &first, sampler &second, const float delta) {
  constexpr auto h = 1e-3f;
  const auto v1 =
      (get_f_opt(first.sample(u + h, v), q_def, P0, tangent, delta) - v_o) *
      (1.f / h);
  const auto v2 =
      (get_f_opt(first.sample(u, v + h), q_def, P0, tangent, delta) - v_o) *
      (1.f / h);
  const auto v3 =
      (get_f_opt(p_def, second.sample(s + h, t), P0, tangent, delta) - v_o) *
      (1.f / h);
  const auto v4 =
      (get_f_opt(p_def, second.sample(s, t + h), P0, tangent, delta) - v_o) *
      (1.f / h);
  return glm::inverse(glm::mat4(v1, v2, v3, v4));
};

auto get_newton_decrement(float u, float v, float s, float t, float delta,
                          sampler &first, sampler &second,
                          const glm::vec3 &tangent, const glm::vec3 &P0,
                          const glm::vec3 &p_def, const glm::vec3 &q_def) {
  auto f = get_f_opt(p_def, q_def, P0, tangent, delta);
  auto inv_j_f =
      get_j(p_def, q_def, u, v, s, t, f, P0, tangent, first, second, delta);
  return inv_j_f * f;
};

inline glm::vec3 get_tangent(float u, float v, float s, float t, sampler &first,
                             sampler &second) {
  auto pnormal = [&](auto u, auto v) {
    return glm::normalize(glm::cross(first.der_u(u, v), first.der_v(u, v)));
  };

  auto qnormal = [&](auto s, auto t) {
    return glm::normalize(glm::cross(second.der_u(s, t), second.der_v(s, t)));
  };

  return glm::normalize(glm::cross(pnormal(u, v), qnormal(s, t)));
}

inline intersection_status
extend_start_point(std::vector<glm::vec3> &points,
                   std::vector<glm::vec4> &coords,
                   const glm::vec4 &start_coords, float delta, sampler &first,
                   sampler &second, const intersection_params &params) {
  auto first_point = points[0];
  auto last_coords = start_coords;
  auto iter = 0;
  while (iter < 20000) {
    ++iter;
    auto P0 = first.sample(last_coords.x, last_coords.y);
    glm::vec4 next_coords = last_coords;
    const auto tangent =
        get_tangent(last_coords.x, last_coords.y, last_coords.z, last_coords.w,
                    first, second);

    // newton away baby
    auto q_def = second.sample(next_coords.z, next_coords.w);
    auto p_def = first.sample(next_coords.x, next_coords.y);

    next_coords.x = wrap(next_coords.x);
    next_coords.y = wrap(next_coords.y);
    next_coords.z = wrap(next_coords.z);
    next_coords.w = wrap(next_coords.w);

    for (int i = 0; i < params.newton_iters; ++i) {
        if (std::isnan(next_coords.x) || std::isnan(next_coords.y) ||
            std::isnan(next_coords.z) || std::isnan(next_coords.w)) {
          return intersection_status::nan_error;
        }

        next_coords -= get_newton_decrement(
          next_coords.x, next_coords.y, next_coords.z, next_coords.w, delta,
          first, second, tangent, P0, p_def, q_def);
      if (glm::length(p_def - q_def) < params.newton_acceptance) {
        break;
      }
    }

    auto last_point = points[points.size() - 1];

    // check if cycle
    if (glm::length(first_point - last_point) < std::abs(delta) &&
        points.size() > 4) {
      points.push_back(first_point);
      break;
    }

    if (std::isnan(next_coords.x) || std::isnan(next_coords.y) ||
        std::isnan(next_coords.z) || std::isnan(next_coords.w)) {
      return intersection_status::nan_error;
    }

    auto fs = first.sample(next_coords.x, next_coords.y);
    auto ss = second.sample(next_coords.z, next_coords.w);
    // Check if Newton is converging
    if ((!first.u_wrapped && needs_wrapping(next_coords.x)) ||
        (!first.v_wrapped && needs_wrapping(next_coords.y)) ||
        (!second.u_wrapped && needs_wrapping(next_coords.z)) ||
        (!second.v_wrapped && needs_wrapping(next_coords.w)) ||
        (glm::length(fs - ss) > std::abs(delta))) {
      break;
    }

    auto pos = first.sample(next_coords.x, next_coords.y);

    if (std::isnan(pos.x) || std::isnan(pos.y) || std::isnan(pos.z)) {
      return intersection_status::nan_error;
    }

    points.push_back(pos);
    coords.push_back(next_coords);
    last_coords = next_coords;
  }
  return intersection_status::success;
}

intersection_status find_all_intersection_points(
    const intersection_params &params, glm::vec4 start_coords,
    std::vector<glm::vec3> &points, std::vector<glm::vec4> &coords,
    sampler &first, sampler &second) {
  auto first_point = points[0];

  // find first line of points (or cycle)
  auto extension_status = extend_start_point(
      points, coords, start_coords, params.delta, first, second, params);
  if (extension_status != intersection_status::success) {
    return extension_status;
  }
  auto last_point = points[points.size() - 1];

  const bool cycle_found =
      (glm::length(first_point - last_point) < std::abs(params.delta) &&
       points.size() > 4);

  // no cycle, we need to do reverse delta now
  std::vector<glm::vec3> tmp_points{first_point};
  std::vector<glm::vec4> tmp_coords{start_coords};
  if (!cycle_found) {
    extend_start_point(tmp_points, tmp_coords, start_coords, -params.delta,
                       first, second, params);
  }

  // combine two parts
  points.insert(points.begin(), tmp_points.rbegin(), tmp_points.rend());
  coords.insert(coords.begin(), tmp_coords.rbegin(), tmp_coords.rend());

  return intersection_status::success;
}

intersect_return intersect(ecs::EntityType first_idx,
                           ecs::EntityType second_idx, sampler &first,
                           sampler &second, const intersection_params &params,
                           bool self_intersection,
                           const glm::vec3 &cursor_pos) {
  auto &reg = ecs::registry::get_registry();
  float su{0.0f}, sv{0.0f}, ss{0.0f}, st{0.0f};

  // find start point
  auto start_point_status = find_start_point(
      params, su, sv, ss, st, first, second, self_intersection, cursor_pos);
  if (self_intersection && std::abs(su - ss) < 0.01 &&
      std::abs(sv - st) < 0.01) {
    return {intersection_status::self_intersection_error, {}};
  }
  if (start_point_status != intersection_status::success) {
    return {start_point_status, {}};
  }

  // add start point
  std::vector<glm::vec3> points{first.sample(su, sv), second.sample(ss, st)};
  std::vector<glm::vec4> coords{glm::vec4{su, sv, ss, st}};
  glm::vec4 start_coords = {su, sv, ss, st};

  // find the entire intersection
  auto newton_status = find_all_intersection_points(
      params, start_coords, points, coords, first, second);

  if (newton_status != intersection_status::success) {
    return {newton_status, {}};
  }

  // add intersection figure
  auto intersection_idx =
      constructors::add_intersection(points, first_idx, second_idx);

  if (!self_intersection) {
    // add textures
    auto &gl1 = reg.get_component<gl_object>(first_idx);
    auto &gl2 = reg.get_component<gl_object>(second_idx);

    GLuint tex[2];
    glCreateTextures(GL_TEXTURE_2D, 2, tex);

    constexpr int texture_size = 512;

    // color
    glTextureParameteri(tex[0], GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTextureParameteri(tex[0], GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTextureParameteri(tex[0], GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTextureParameteri(tex[0], GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTextureStorage2D(tex[0], 1, GL_R8, texture_size, texture_size);

    glTextureParameteri(tex[1], GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTextureParameteri(tex[1], GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTextureParameteri(tex[1], GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTextureParameteri(tex[1], GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTextureStorage2D(tex[1], 1, GL_R8, texture_size, texture_size);

    float clear = 0.f;
    glClearTexSubImage(tex[0], 0, 0, 0, 0, texture_size, texture_size, 1,
                       GL_RED, GL_FLOAT, &clear);
    glClearTexSubImage(tex[1], 0, 0, 0, 0, texture_size, texture_size, 1,
                       GL_RED, GL_FLOAT, &clear);

    // fill texture with lines
    std::array<float, texture_size * texture_size> tex_cpu;
    tex_cpu.fill(0.f);

    auto set_pixel = [&](int x, int y, float val) {
      if (x >= texture_size || y >= texture_size) {
        return;
      }
      tex_cpu[y * texture_size + x] = val;
    };

    auto get_pixel = [&](int x, int y) {
      return tex_cpu[y * texture_size + x];
    };

    // BRASENHAM FILLING HERE
    // FIRST TEXTURE
    for (std::size_t seg = 1; seg < coords.size(); ++seg) {
      auto fc1 = coords[seg - 1];
      auto first_c =
          glm::vec4{wrap(fc1.x), wrap(fc1.y), wrap(fc1.z), wrap(fc1.w)};
      auto second_c = coords[seg];

      Bresenham<texture_size>(first_c.x, first_c.y, second_c.x, second_c.y,
                              set_pixel);
      if (needs_wrapping(second_c.x) || needs_wrapping(second_c.y)) {
        if (second_c.x < 0.f) {
          Bresenham<texture_size>(first_c.x + 1.f, first_c.y, second_c.x + 1.f,
                                  second_c.y, set_pixel);
        } else if (second_c.x > 1.f) {
          Bresenham<texture_size>(first_c.x - 1.f, first_c.y, second_c.x - 1.f,
                                  second_c.y, set_pixel);
        }

        if (second_c.y < 0.f) {
          Bresenham<texture_size>(first_c.x, first_c.y + 1.f, second_c.x,
                                  second_c.y + 1.f, set_pixel);
        } else if (second_c.y > 1.f) {
          Bresenham<texture_size>(first_c.x, first_c.y - 1.f, second_c.x,
                                  second_c.y - 1.f, set_pixel);
        }
      }
    }

    // flood fill
    flood_fill<texture_size>(first.u_wrapped, first.v_wrapped, get_pixel,
                             set_pixel);

    glTextureSubImage2D(tex[0], 0, 0, 0, texture_size, texture_size, GL_RED,
                        GL_FLOAT, tex_cpu.data());
    gl1.trim_texture = tex[0];
    gl1.trimming_info = {1.0f, 1.0f, 0.0f, 0.0f};

    // SECOND TEXTURE
    tex_cpu.fill(0.f);
    for (std::size_t seg = 1; seg < coords.size(); ++seg) {
      auto fc1 = coords[seg - 1];
      auto first_c =
          glm::vec4{wrap(fc1.x), wrap(fc1.y), wrap(fc1.z), wrap(fc1.w)};
      auto second_c = coords[seg];

      Bresenham<texture_size>(first_c.z, first_c.w, second_c.z, second_c.w,
                              set_pixel);
      if (needs_wrapping(second_c.z) || needs_wrapping(second_c.w)) {
        if (second_c.z < 0.f) {
          Bresenham<texture_size>(first_c.z + 1.f, first_c.w, second_c.z + 1.f,
                                  second_c.w, set_pixel);
        } else if (second_c.z > 1.f) {
          Bresenham<texture_size>(first_c.z - 1.f, first_c.w, second_c.z - 1.f,
                                  second_c.w, set_pixel);
        }

        if (second_c.w < 0.f) {
          Bresenham<texture_size>(first_c.z, first_c.w + 1.f, second_c.z,
                                  second_c.w + 1.f, set_pixel);
        } else if (second_c.w > 1.f) {
          Bresenham<texture_size>(first_c.z, first_c.w - 1.f, second_c.z,
                                  second_c.w - 1.f, set_pixel);
        }
      }
    }

    // flood fill
    flood_fill<texture_size>(second.u_wrapped, second.v_wrapped, get_pixel,
                             set_pixel);

    glTextureSubImage2D(tex[1], 0, 0, 0, texture_size, texture_size, GL_RED,
                        GL_FLOAT, tex_cpu.data());
    gl2.trim_texture = tex[1];
    gl2.trimming_info = {1.0f, 1.0f, 0.0f, 0.0f};
  }

  return {intersection_status::success, intersection_idx};
}
} // namespace systems
