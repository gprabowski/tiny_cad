#include <bezier_surface.h>
#include <systems.h>

namespace systems {
void regenerate_bezier_surface(ecs::EntityType idx) {
  // based on relationship create vao and vbo
  static auto &reg = ecs::registry::get_registry();

  auto &bsp = reg.get_component<bezier_surface_params>(idx);
  auto &g = reg.get_component<gl_object>(idx);
  auto &rel = reg.get_component<relationship>(idx);

  g.points.clear();
  g.indices.clear();
  // build points
  for (auto c : rel.children) {
    auto &t = reg.get_component<transformation>(c);
    g.points.push_back(glm::vec4(t.translation, 1.0f));
  }
  // build indices
  for (unsigned int j = 0; j < bsp.v; ++j) {
    for (unsigned int i = 0; i < bsp.u; ++i) {
      if (!bsp.cyllinder) {
        for (auto jj = 0; jj < 4; ++jj) {
          for (auto ii = 0; ii < 4; ++ii) {
            g.indices.push_back(3 * ((3 * bsp.u + 1) * j + i) +
                                (3 * bsp.u + 1) * jj + ii);
          }
        }
      } else {
        for (auto jj = 0; jj < 4; ++jj) {
          for (auto ii = 0; ii < 4; ++ii) {
            const auto patch_col_offset = 3 * ((3 * bsp.u) * j);
            const auto patch_row_offset = 3 * i;
            const auto local_col_offset = (3 * bsp.u) * jj;
            const auto local_row_offset = ii;
            g.indices.push_back(patch_col_offset + local_col_offset +
                                (patch_row_offset + local_row_offset) %
                                    (bsp.u * 3));
          }
        }
      }
    }
  }
  g.dmode = gl_object::draw_mode::patches;
  g.patch_size = 16;
  systems::reset_gl_objects(g);
}

void regenerate_bezier_surface_builder(ecs::EntityType idx) {
  static auto &reg = ecs::registry::get_registry();

  auto &g = reg.get_component<gl_object>(idx);
  auto &bsp = reg.get_component<bezier_surface_params>(idx);
  g.points.clear();
  g.indices.clear();

  std::vector<glm::vec3> points;

  if (!bsp.cyllinder) {
    for (unsigned int i = 0; i <= bsp.u * 3u; ++i) {
      points.push_back({bsp.root + glm::vec3{bsp.width / 3.f * i, 0.f, 0.f}});
      points.push_back(
          {bsp.root + glm::vec3{bsp.width / 3.f * i, 0.f, bsp.height * bsp.v}});
    }

    for (unsigned int j = 0; j <= bsp.v * 3u; ++j) {
      points.push_back({bsp.root + glm::vec3{0.f, 0.f, bsp.height / 3.f * j}});
      points.push_back(
          {bsp.root + glm::vec3{bsp.width * bsp.u, 0.f, bsp.height / 3.f * j}});
    }

    for (std::size_t p = 0u; p < points.size(); ++p) {
      g.points.push_back(glm::vec4(points[p], 1.0f));
      g.indices.push_back(p);
    }

  } else {
    const auto angle = (2 * glm::pi<float>()) / bsp.u;
    const auto dist =
        bsp.width * (4.f / 3.f) * tanf(glm::pi<float>() / (2.f * bsp.u));
    for (unsigned int j = 0; j <= bsp.v; ++j) {
      for (unsigned int i = 0; i < bsp.u; ++i) {
        // first rooted on the circle
        const auto tan1 =
            glm::normalize(glm::vec3{-sinf(i * angle), 0.0f, cosf(i * angle)});
        const auto tan2 = glm::normalize(
            glm::vec3{-sinf((i + 1) * angle), 0.f, cosf((i + 1) * angle)});

        const auto p1 = (bsp.root + glm::vec3{bsp.width * cosf(i * angle),
                                              j * bsp.height / 4.f,
                                              bsp.width * sinf(i * angle)});

        const auto p2 = p1 + dist * tan1;

        // second rooted on the circle
        const auto p4 =
            (bsp.root + glm::vec3{bsp.width * cosf((i + 1) * angle),
                                  j * bsp.height / 4.f,
                                  bsp.width * sinf((i + 1) * angle)});

        const auto p3 = p4 - dist * tan2;

        points.push_back(p1);
        points.push_back(p2);
        points.push_back(p3);
        points.push_back(p4);
      }
    }

    g.points.push_back(glm::vec4(points[0], 1.0f));
    for (std::size_t p = 1u; p < points.size(); ++p) {
      g.points.push_back(glm::vec4(points[p], 1.0f));
      g.indices.push_back(p - 1);
      g.indices.push_back(p);
    }
  }

  reset_gl_objects(g);
}
} // namespace systems
