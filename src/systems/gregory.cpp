#include <systems.h>

#include <registry.h>

namespace systems {

glm::vec3 casteljau(float t, glm::vec3 &a, glm::vec3 &b, glm::vec3 &c,
                    glm::vec3 &d) {
  glm::vec3 e = (1.f - t) * a + t * b;
  glm::vec3 f = (1.f - t) * b + t * c;
  glm::vec3 g = (1.f - t) * c + t * d;

  glm::vec3 h = (1.f - t) * e + t * f;
  glm::vec3 i = (1.f - t) * f + t * g;

  return (1.f - t) * h + t * i;
}

glm::vec3 casteljau(float t, glm::vec3 &e, glm::vec3 &f, glm::vec3 &g) {
  glm::vec3 h = (1.f - t) * e + t * f;
  glm::vec3 i = (1.f - t) * f + t * g;

  return (1.f - t) * h + t * i;
}

void find_combination(glm::vec3 a, glm::vec3 b, glm::vec3 res, float &u,
                      float &v) {
  glm::mat3 ext{a, b, res};
  ext = glm::transpose(ext);
  // gauss first row
  ext[0] = ext[0] / ext[0][0];
  ext[1] = ext[1] - ext[0] * (ext[1][0] / ext[0][0]);
  ext[2] = ext[2] - ext[0] * (ext[2][0] / ext[0][0]);

  ext[1] = ext[1] / ext[1][1];
  ext[2] = ext[2] - ext[1] * (ext[2][1] / ext[1][1]);

  // ready
  if (glm::length(ext[2]) > 0.1) {
    throw;
  }

  ext = glm::transpose(ext);
  v = ext[2][1];
  u = ext[2][0] - ext[1][0] * v;
}

void regenerate_gregory(ecs::EntityType idx) {
  auto &reg = ecs::registry::get_registry();
  auto &rel = reg.get_component<relationship>(idx);
  auto &rc = rel.children;
  auto &g = reg.get_component<gl_object>(idx);
  auto &tg = reg.get_component<tag_gregory>(idx);
  auto &dg = reg.get_component<gl_object>(tg.derivatives);

  g.points.clear();
  g.indices.clear();
  dg.points.clear();
  dg.indices.clear();

  // 1. get actual points from the diagram by dividing the first layer
  // in half 3 times
  auto gt = [&](auto idx) -> glm::vec3 {
    transformation &t = reg.get_component<transformation>(idx);
    return t.translation;
  };

  std::array<glm::vec3, 49> p;

  for (int i = 0; i < 3; ++i) {
    std::array<glm::vec3, 6> tmp;
    glm::vec3 a{gt(rc[8 * i])}, b{gt(rc[8 * i + 1])}, c{gt(rc[8 * i + 2])},
        d{gt(rc[8 * i + 3])};
    tmp[0] = (a + b) * 0.5f;
    tmp[1] = (b + c) * 0.5f;
    tmp[2] = (c + d) * 0.5f;
    tmp[3] = (tmp[0] + tmp[1]) * 0.5f;
    tmp[4] = (tmp[1] + tmp[2]) * 0.5f;
    tmp[5] = (tmp[3] + tmp[4]) * 0.5f;

    p[i * 6] = a;
    p[i * 6 + 1] = tmp[0];
    p[i * 6 + 2] = tmp[3];
    p[i * 6 + 3] = tmp[5];
    p[i * 6 + 4] = tmp[4];
    p[i * 6 + 5] = tmp[2];
    p[(i * 6 + 6) % 18] = d;
  }

  // 2. get second layer of points by dividing the second layer in half and
  // then subtracting proper points from each other
  std::array<glm::vec3, 21> behind;
  for (int i = 0; i < 3; ++i) {
    std::array<glm::vec3, 6> tmp;
    glm::vec3 a{gt(rc[8 * i + 4])}, b{gt(rc[8 * i + 1 + 4])},
        c{gt(rc[8 * i + 2 + 4])}, d{gt(rc[8 * i + 3 + 4])};
    tmp[0] = (a + b) * 0.5f;
    tmp[1] = (b + c) * 0.5f;
    tmp[2] = (c + d) * 0.5f;
    tmp[3] = (tmp[0] + tmp[1]) * 0.5f;
    tmp[4] = (tmp[1] + tmp[2]) * 0.5f;
    tmp[5] = (tmp[3] + tmp[4]) * 0.5f;

    behind[i * 7] = a;
    behind[i * 7 + 1] = tmp[0];
    behind[i * 7 + 2] = tmp[3];
    behind[i * 7 + 3] = tmp[5];
    behind[i * 7 + 4] = tmp[4];
    behind[i * 7 + 5] = tmp[2];
    behind[i * 7 + 6] = d;
  }

  // obvious inner points
  for (int i = 0; i < 3; ++i) {
    p[18 + 7 * i] = p[1 + 6 * i] + (p[1 + 6 * i] - behind[1 + 7 * i]);
    p[19 + 7 * i] = p[2 + 6 * i] + (p[2 + 6 * i] - behind[2 + 7 * i]);
    p[21 + 7 * i] = p[3 + 6 * i] + (p[3 + 6 * i] - behind[3 + 7 * i]);
    p[23 + 7 * i] = p[4 + 6 * i] + (p[4 + 6 * i] - behind[4 + 7 * i]);
    p[24 + 7 * i] = p[5 + 6 * i] + (p[5 + 6 * i] - behind[5 + 7 * i]);
  }

  {
    std::array<glm::vec3, 3> q{};
    for (int i = 0; i < 3; ++i) {
      q[i] = p[3 + 6 * i] + (3.f / 2.f) * (p[3 + 6 * i] - behind[3 + 7 * i]);
    }

    p[48] = (q[0] + q[1] + q[2]) / 3.f;

    for (int i = 0; i < 3; ++i) {
      p[40 + i * 3] = q[i] + (p[48] - q[i]) * (1.f / 3.f);
    }
  }

  // now for all 3 connecting borders
  uint32_t ha2[3] = {46, 40, 43};
  uint32_t hb2[3] = {43, 46, 40};

  for (int i = 0; i < 3; ++i) {
    auto _a0_1 = p[3 + 6 * i] - p[2 + 6 * i];
    auto _b0_1 = p[4 + 6 * i] - p[3 + 6 * i];
    auto _a2_1 = p[48] - p[ha2[i]];
    auto _b2_1 = p[hb2[i]] - p[48];

    struct two_d {
      glm::vec3 u, v;
    };

    auto find_two_d = [&](auto &a0, auto &b0, auto &a2, auto &b2) -> two_d {
      auto g0 = ((a0 + b0) / 2.f);
      auto g2 = ((a2 + b2) / 2.f);
      auto g1 = (g0 + g2) / 2.f;
      auto c0 = (p[21 + 7 * i] - p[3 + 6 * i]);
      auto c1 = (p[40 + 3 * i] - p[21 + 7 * i]);
      auto c2 = (p[48] - p[40 + 3 * i]);

      auto g_func = [&](auto t) { return casteljau(t, g0, g1, g2); };
      auto c_func = [&](auto t) { return casteljau(t, c0, c1, c2); };

      float k0, h0, k1, h1;

      find_combination(g0, c0, b0, k0, h0);
      find_combination(g2, c2, b2, k1, h1);

      auto k_func = [&](auto t) { return k0 * (1 - t) + k1 * t; };
      auto h_func = [&](auto t) { return h0 * (1 - t) + h1 * t; };

      auto d_func = [&](auto t) {
        return k_func(t) * g_func(t) + h_func(t) * c_func(t);
      };

      return {d_func(1.f / 3.f), d_func(2.f / 3.f)};
    };

    auto res1 = find_two_d(_a0_1, _b0_1, _a2_1, _b2_1);

    p[20 + 7 * i] = p[21 + 7 * i] - res1.u;
    p[39 + 3 * i] = p[40 + 3 * i] - res1.v;

    p[22 + 7 * i] = p[21 + 7 * i] + res1.u;
    p[41 + 3 * i] = p[40 + 3 * i] + res1.v;

    p[20 + 7 * i] = p[19 + 7 * i];
    p[22 + 7 * i] = p[23 + 7 * i];

    for (int k = 1; k < 7; ++k) {
      dg.indices.push_back(k - 1 + 6 * i);
      dg.indices.push_back((k + 6 * i) % 18);
    }

    {
      auto _tmp = 1;
      for (auto _u : {18, 19, 21, 23, 24}) {
        dg.indices.push_back((_tmp++) + 6 * i);
        dg.indices.push_back(_u + 7 * i);
      }
    }

    dg.indices.push_back(21 + 7 * i);
    dg.indices.push_back(20 + 7 * i);

    dg.indices.push_back(21 + 7 * i);
    dg.indices.push_back(22 + 7 * i);

    dg.indices.push_back(40 + 3 * i);
    dg.indices.push_back(21 + 7 * i);

    dg.indices.push_back(40 + 3 * i);
    dg.indices.push_back(39 + 3 * i);

    dg.indices.push_back(40 + 3 * i);
    dg.indices.push_back(41 + 3 * i);

    dg.indices.push_back(40 + 3 * i);
    dg.indices.push_back(48);
  }

  for (auto _p : p) {
    g.points.push_back(glm::vec4(_p, 1.0f));
    dg.points.push_back(glm::vec4(_p, 1.0f));
  }

  g.indices = {
      // first
      15, 35, 46, 48,         //
      16, 36, 37, 47, 39, 40, //
      17, 38, 18, 20, 19, 21, //
      0, 1, 2, 3,
      // second
      3, 21, 40, 48,         //
      4, 22, 23, 41, 42, 43, //
      5, 24, 25, 27, 26, 28, //
      6, 7, 8, 9,            //
      // third
      9, 28, 43, 48,          //
      10, 29, 30, 44, 45, 46, //
      11, 31, 32, 34, 33, 35, //
      12, 13, 14, 15          //
  };

  reset_gl_objects(g);
  reset_gl_objects(dg);
}
} // namespace systems
