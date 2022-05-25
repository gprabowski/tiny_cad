#include <bspline_surface.h>
#include <systems.h>

namespace systems {
void regenerate_bspline_surface(ecs::EntityType idx) {
  // based on relationship create vao and vbo
  static auto &reg = ecs::registry::get_registry();

  auto &bsp = reg.get_component<bspline_surface_params>(idx);
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
            const auto patch_col_offset = j * (4 + bsp.u - 1);
            const auto patch_row_offset = i;
            const auto local_col_offset = (4 + bsp.u - 1) * (jj);
            const auto local_row_offset = ii;
            g.indices.push_back(patch_col_offset + local_col_offset +
                                patch_row_offset + local_row_offset);
          }
        }
      } else {
        for (auto jj = 0; jj < 4; ++jj) {
          for (auto ii = 0; ii < 4; ++ii) {
            const auto patch_col_offset = j * (4 + bsp.u - 1 - 3);
            const auto patch_row_offset = i;
            const auto local_col_offset = (4 + bsp.u - 1 - 3) * (jj);
            const auto local_row_offset = ii;
            g.indices.push_back(patch_col_offset + local_col_offset +
                                (patch_row_offset + local_row_offset) %
                                    (4 + bsp.u - 1 - 3));
          }
        }
      }
    }
  }
  g.dmode = gl_object::draw_mode::patches;
  g.patch_size = 16;

  // bezier polygon
  auto &bgl = reg.get_component<gl_object>(bsp.deboor_polygon);
  bgl.points.clear();
  bgl.indices.clear();

  if (!bsp.cyllinder) {
    // vertices
    for (unsigned int j = 0; j < 4 + bsp.v - 1; ++j) {
      for (unsigned int i = 0; i < 4 + bsp.u - 1; ++i) {
        bgl.points.push_back(g.points[j * (4 + bsp.u - 1) + i]);
        if (i > 0) {
          bgl.indices.push_back(j * (4 + bsp.u - 1) + i - 1);
          bgl.indices.push_back(j * (4 + bsp.u - 1) + i);
        }
      }
    }
    for (unsigned int i = 0; i < (4 + bsp.u - 1); ++i) {
      for (unsigned int j = 0; j < (4 + bsp.v - 1); ++j) {
        if (j > 0) {
          bgl.indices.push_back((j - 1) * (4 + bsp.u - 1) + i);
          bgl.indices.push_back(j * (4 + bsp.u - 1) + i);
        }
      }
    }
  } else {
    for (unsigned int j = 0; j <= 3 * bsp.v; ++j) {
      for (unsigned int i = 0; i <= 3 * bsp.u; ++i) {
        bgl.points.push_back(g.points[j * (3 * bsp.u + 1) + i]);
        if (i > 0) {
          bgl.indices.push_back(j * (3 * bsp.u) + (i - 1) % (bsp.u * 3));
          bgl.indices.push_back(j * (3 * bsp.u) + (i) % (bsp.u * 3));
        }
      }
    }
    for (unsigned int i = 0; i < 3 * bsp.u; ++i) {
      for (unsigned int j = 0; j <= 3 * bsp.v; ++j) {
        if (j > 0) {
          bgl.indices.push_back((j - 1) * (3 * bsp.u) + i);
          bgl.indices.push_back(j * (3 * bsp.u) + i);
        }
      }
    }
  }

  systems::reset_gl_objects(g);
  systems::reset_gl_objects(bgl);
}

void regenerate_bspline_surface_builder(ecs::EntityType idx) {
  static auto &reg = ecs::registry::get_registry();

  auto &g = reg.get_component<gl_object>(idx);
  auto &bsp = reg.get_component<bspline_surface_params>(idx);
  g.points.clear();
  g.indices.clear();

  std::vector<glm::vec3> points;

  if (!bsp.cyllinder) {
    for (unsigned int i = 0; i < 4 + bsp.u - 1; ++i) {
      points.push_back({bsp.root + glm::vec3{bsp.width / 3.f * i, 0.f, 0.f}});
      points.push_back(
          {bsp.root + glm::vec3{bsp.width / 3.f * i, 0.f,
                                bsp.height / 3.f * (3 + bsp.v - 1)}});
    }

    for (unsigned int j = 0; j < 4 + bsp.v - 1; ++j) {
      points.push_back({bsp.root + glm::vec3{0.f, 0.f, bsp.height / 3.f * j}});
      points.push_back({bsp.root + glm::vec3{bsp.width / 3.f * (3 + bsp.u - 1),
                                             0.f, bsp.height / 3.f * j}});
    }

    for (std::size_t p = 0u; p < points.size(); ++p) {
      g.points.push_back(glm::vec4(points[p], 1.0f));
      g.indices.push_back(p);
    }

  } else {
    const auto angle = (2 * glm::pi<float>()) / (4 + bsp.u - 1 - 3);
    for (unsigned int j = 0; j < 4 + bsp.v - 1; ++j) {
      for (unsigned int i = 0; i < 4 + bsp.u; ++i) {
        // first rooted on the circle
        const auto p1 = (bsp.root + glm::vec3{bsp.width * cosf(i * angle),
                                              j * bsp.height / 3.f,
                                              bsp.width * sinf(i * angle)});

        points.push_back(p1);
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
