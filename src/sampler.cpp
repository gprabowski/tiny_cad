#include <registry.h>
#include <sampler.h>

#define GLM_FORCE_RADIANS
#include <glm/ext/matrix_transform.hpp>
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>

int get_bezier_pos(int i, int j, int ii, int jj, int bspu, bool cyllinder) {
  int ret;
  if (!cyllinder) {
    auto tmp_idx = (3 * ((3 * bspu + 1) * j + i) + (3 * bspu + 1) * jj + ii);
    ret = tmp_idx;
  } else {
    const auto patch_col_offset = 3 * ((3 * bspu) * j);
    const auto patch_row_offset = 3 * i;
    const auto local_col_offset = (3 * bspu) * jj;
    const auto local_row_offset = ii;
    auto tmp_idx = (patch_col_offset + local_col_offset +
                    (patch_row_offset + local_row_offset) % (bspu * 3));
    ret = tmp_idx;
  }

  return ret;
}

int get_bspline_pos(int i, int j, int ii, int jj, bspline_surface_params &bsp) {
  int ret;
  if (!bsp.cyllinder) {
    const auto patch_col_offset = j * (4 + bsp.u - 1);
    const auto patch_row_offset = i;
    const auto local_col_offset = (4 + bsp.u - 1) * (jj);
    const auto local_row_offset = ii;
    const auto idx = (patch_col_offset + local_col_offset + patch_row_offset +
                      local_row_offset);
    ret = (idx);
  } else {
    const auto patch_col_offset = j * (4 + bsp.u - 1 - 3);
    const auto patch_row_offset = i;
    const auto local_col_offset = (4 + bsp.u - 1 - 3) * (jj);
    const auto local_row_offset = ii;
    const auto idx =
        (patch_col_offset + local_col_offset +
         (patch_row_offset + local_row_offset) % (4 + bsp.u - 1 - 3));
    ret = idx;
  }
  return ret;
}

glm::vec3 deboor(float t, const std::array<glm::vec3, 3> &ms) {
  const glm::vec3 &P10 = ms[0];
  const glm::vec3 &P20 = ms[1];
  const glm::vec3 &P30 = ms[2];

  const float u = 1.f + t;

  const float a31 = (u - 1.0f) / 2.0f;
  const float a21 = (u) / 2.0f;

  const float b31 = 1.0f - a31;
  const float b21 = 1.0f - a21;

  const glm::vec3 P31 = a31 * P30 + b31 * P20;
  const glm::vec3 P21 = a21 * P20 + b21 * P10;

  const float a32 = (u - 1.0f) / 1.0f;
  const float b32 = 1.0f - a32;

  const glm::vec3 P32 = a32 * P31 + b32 * P21;

  return P32;
}

glm::vec3 deboor(float t, glm::vec3 P10, glm::vec3 P20, glm::vec3 P30,
                 glm::vec3 P40) {
  const float u = 2.f + t;

  const float a41 = (u - 2.0f) / 3.0f;
  const float a31 = (u - 1.0f) / 3.0f;
  const float a21 = (u) / 3.0f;

  const float b41 = 1.0f - a41;
  const float b31 = 1.0f - a31;
  const float b21 = 1.0f - a21;

  const glm::vec3 P41 = a41 * P40 + b41 * P30;
  const glm::vec3 P31 = a31 * P30 + b31 * P20;
  const glm::vec3 P21 = a21 * P20 + b21 * P10;

  const float a42 = (u - 2.0f) / 2.0f;
  const float a32 = (u - 1.0f) / 2.0f;

  const float b42 = 1.0f - a42;
  const float b32 = 1.0f - a32;

  const glm::vec3 P42 = a42 * P41 + b42 * P31;
  const glm::vec3 P32 = a32 * P31 + b32 * P21;

  const float a43 = (u - 2.0f) / 1.0f;
  const float b43 = 1.0f - a43;

  const glm::vec3 P43 = a43 * P42 + b43 * P32;

  return P43;
}

glm::vec3 casteljau(float t, glm::vec3 e, glm::vec3 f, glm::vec3 g) {
  glm::vec3 h = (1.0f - t) * e + t * f;
  glm::vec3 i = (1.0f - t) * f + t * g;

  return (1.0f - t) * h + t * i;
}

glm::vec3 casteljau(float t, glm::vec3 a, glm::vec3 b, glm::vec3 c,
                    glm::vec3 d) {
  glm::vec3 e = (1.0f - t) * a + t * b;
  glm::vec3 f = (1.0f - t) * b + t * c;
  glm::vec3 g = (1.0f - t) * c + t * d;

  glm::vec3 h = (1.0f - t) * e + t * f;
  glm::vec3 i = (1.0f - t) * f + t * g;

  return (1.0f - t) * h + t * i;
}

sampler get_sampler(ecs::EntityType idx) {
  auto &reg = ecs::registry::get_registry();
  sampler s;
  if (reg.has_component<torus_params>(idx)) {
    auto &tp = reg.get_component<torus_params>(idx);
    auto &t = reg.get_component<transformation>(idx);
    s.u_wrapped = true;
    s.v_wrapped = true;

    const auto trans = glm::translate(glm::mat4(1.0f), t.translation);
    const auto scale = glm::scale(glm::mat4(1.0f), t.scale);
    const auto rot = glm::toMat4(glm::quat(glm::radians(t.rotation)));

    const auto model = trans * scale * rot;
    const auto model_without_trans = scale * rot;

    s.sample = [=](float u, float v) -> glm::vec3 {
      u = glm::pi<float>() * 2.f * u;
      v = glm::pi<float>() * 2.f * v;

      const auto sin_u = sinf(u);
      const auto cos_u = cosf(u);

      const auto sin_v = sinf(v);
      const auto cos_v = cosf(v);

      return glm::vec3(
          model * glm::vec4{tp.radii[1] * cos_u + tp.radii[0] * cos_u * cos_v,
                            tp.radii[0] * sin_v,
                            tp.radii[1] * sin_u + tp.radii[0] * sin_u * cos_v,
                            1.0f});
    };

    s.der_u = [=](float u, float v) -> glm::vec3 {
      u = glm::pi<float>() * 2.f * u;
      v = glm::pi<float>() * 2.f * v;

      const auto sin_u = sinf(u);
      const auto cos_u = cosf(u);

      const auto cos_v = cosf(v);

      return glm::vec3(
          model_without_trans *
          glm::vec4{-tp.radii[1] * sin_u - tp.radii[0] * sin_u * cos_v, 0,
                    tp.radii[1] * cos_u + tp.radii[0] * cos_u * cos_v, 0.0f});
    };

    s.der_u_opt = [=](float u, float v, const glm::vec3 &v_o) -> glm::vec3 {
      return s.der_u(u, v);
    };

    s.der_v = [=](float u, float v) -> glm::vec3 {
      u = glm::pi<float>() * 2.f * u;
      v = glm::pi<float>() * 2.f * v;

      const auto sin_u = sinf(u);
      const auto cos_u = cosf(u);

      const auto sin_v = sinf(v);
      const auto cos_v = cosf(v);

      return glm::vec3(model_without_trans *
                       glm::vec4{-tp.radii[0] * cos_u * sin_v,
                                 tp.radii[0] * cos_v,
                                 -tp.radii[0] * sin_u * sin_v, 0.0f});
    };

    s.der_v_opt = [=](float u, float v, const glm::vec3 &v_o) -> glm::vec3 {
      return s.der_v(u, v);
    };
  }

  if (reg.has_component<bezier_surface_params>(idx)) {
    auto &bsp = reg.get_component<bezier_surface_params>(idx);
    auto &rel = reg.get_component<relationship>(idx);

    s.u_wrapped = bsp.cyllinder;
    s.v_wrapped = false;

    std::vector<glm::vec3> cpoint_pos;
    for (auto c : rel.children) {
      auto &t = reg.get_component<transformation>(c);
      cpoint_pos.push_back(t.translation);
    }

    s.sample = [=](float u, float v) -> glm::vec3 {
      const auto _bsp = bsp;
      unsigned int i, j;
      float uintpart;
      float vintpart;

      float lu = (s.u_wrapped && u > 1.0f) ? std::modf(u, &uintpart) : u;
      float lv = (s.v_wrapped && v > 1.0f) ? std::modf(v, &vintpart) : v;
      // 1. find out which patch parameters belong to
      i = (u == 1.0f) ? bsp.u - 1 : lu * _bsp.u;
      j = (v == 1.0f) ? bsp.v - 1 : lv * _bsp.v;

      // u and v have to be from 0 to 1.0
      // and thus we have to scale them
      lu = _bsp.u * (lu - i * (1.f / _bsp.u));
      lv = _bsp.v * (lv - j * (1.f / _bsp.v));

      // 2. sample that patch
      std::array<glm::vec3, 16> lp;
      for (int jj = 0; jj < 4; ++jj) {
        for (int ii = 0; ii < 4; ++ii) {
          const auto offset =
              get_bezier_pos(i, j, ii, jj, _bsp.u, _bsp.cyllinder);
          lp[jj * 4 + ii] = cpoint_pos[offset];
        }
      }

      auto ctmp1 = casteljau(lu, lp[0], lp[1], lp[2], lp[3]);
      auto ctmp2 = casteljau(lu, lp[4], lp[5], lp[6], lp[7]);
      auto ctmp3 = casteljau(lu, lp[8], lp[9], lp[10], lp[11]);
      auto ctmp4 = casteljau(lu, lp[12], lp[13], lp[14], lp[15]);

      auto tmp_f = casteljau(lv, ctmp1, ctmp2, ctmp3, ctmp4);

      return tmp_f;
    };

    s.der_u = [=](float u, float v) -> glm::vec3 {
      const auto h = 1e-3f;
      const auto v_o = s.sample(u, v);
      const auto v_u = s.sample(u + h, v);
      return (v_u - v_o) * (1.f / h) / static_cast<float>(bsp.u);
    };

    s.der_u_opt = [=](float u, float v, const glm::vec3 &v_o) -> glm::vec3 {
      const auto h = 1e-3f;
      const auto v_u = s.sample(u + h, v);
      return (v_u - v_o) * (1.f / h) / static_cast<float>(bsp.u);
    };

    s.der_v = [=](float u, float v) -> glm::vec3 {
      const auto h = 1e-3f;
      const auto v_o = s.sample(u, v);
      const auto v_v = s.sample(u, v + h);
      return (v_v - v_o) * (1.f / h) / static_cast<float>(bsp.v);
    };

    s.der_v_opt = [=](float u, float v, const glm::vec3 &v_o) -> glm::vec3 {
      const auto h = 1e-3f;
      const auto v_v = s.sample(u, v + h);
      return (v_v - v_o) * (1.f / h) / static_cast<float>(bsp.v);
    };
  }

  if (reg.has_component<bspline_surface_params>(idx)) {
    auto &bsp = reg.get_component<bspline_surface_params>(idx);
    auto &rel = reg.get_component<relationship>(idx);
    s.u_wrapped = bsp.cyllinder;
    s.v_wrapped = false;

    std::vector<glm::vec3> cpoint_pos;
    for (auto c : rel.children) {
      auto &t = reg.get_component<transformation>(c);
      cpoint_pos.push_back(t.translation);
    }

    s.sample = [=](float u, float v) -> glm::vec3 {
      float uintpart;
      float vintpart;

      auto _bsp = bsp;
      int i, j;

      float lu = (s.u_wrapped && u > 1.0f) ? std::modf(u, &uintpart) : u;
      float lv = (s.v_wrapped && v > 1.0f) ? std::modf(v, &vintpart) : v;

      // 1. find out which patch parameters belong to
      i = (u == 1.0f) ? bsp.u - 1 : lu * _bsp.u;
      j = (v == 1.0f) ? bsp.v - 1 : lv * _bsp.v;

      // u and v have to be from 0 to 1.0
      // and thus we have to scale them
      lu = _bsp.u * (lu - i * (1.f / _bsp.u));
      lv = _bsp.v * (lv - j * (1.f / _bsp.v));

      // 2. sample that patch
      std::array<glm::vec3, 16> lp;
      for (int jj = 0; jj < 4; ++jj) {
        for (int ii = 0; ii < 4; ++ii) {
          lp[jj * 4 + ii] = cpoint_pos[get_bspline_pos(i, j, ii, jj, _bsp)];
        }
      }

      auto ctmp1 = deboor(lv, lp[0], lp[4], lp[8], lp[12]);
      auto ctmp2 = deboor(lv, lp[1], lp[5], lp[9], lp[13]);
      auto ctmp3 = deboor(lv, lp[2], lp[6], lp[10], lp[14]);
      auto ctmp4 = deboor(lv, lp[3], lp[7], lp[11], lp[15]);

      auto tmp_f = deboor(lu, ctmp1, ctmp2, ctmp3, ctmp4);

      return tmp_f;
    };

    s.der_u = [=](float u, float v) -> glm::vec3 {
      const auto h = 1e-3f;
      const auto v_o = s.sample(u, v);
      const auto v_u = s.sample(u + h, v);
      return (v_u - v_o) * (1.f / h) / static_cast<float>(bsp.u);
    };

    s.der_u_opt = [=](float u, float v, const glm::vec3 &v_o) -> glm::vec3 {
      const auto h = 1e-3f;
      const auto v_u = s.sample(u + h, v);
      return (v_u - v_o) * (1.f / h) / static_cast<float>(bsp.u);
    };

    s.der_v = [=](float u, float v) -> glm::vec3 {
      const auto h = 1e-3f;
      const auto v_o = s.sample(u, v);
      const auto v_v = s.sample(u, v + h);
      return (v_v - v_o) * (1.f / h) / static_cast<float>(bsp.v);
    };

    s.der_v_opt = [=](float u, float v, const glm::vec3 &v_o) -> glm::vec3 {
      const auto h = 1e-3f;
      const auto v_v = s.sample(u, v + h);
      return (v_v - v_o) * (1.f / h) / static_cast<float>(bsp.v);
    };
  }

  return s;
}
