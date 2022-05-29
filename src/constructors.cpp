#include <bezier_surface.h>
#include <constructors.h>
#include <log.h>
#include <relationship.h>
#include <shader_manager.h>
#include <systems.h>

#include <set>

namespace constructors {

ecs::EntityType add_cursor(transformation &&_t, gl_object &&_g) {
  auto &reg = ecs::registry::get_registry();
  const auto t = reg.add_entity();
  reg.add_component<transformation>(t, std::move(_t));
  reg.add_component<gl_object>(t, std::move(_g));
  reg.add_component<cursor_params>(t, cursor_params{});
  reg.add_component<tag_visible>(t, {});

  auto &tr = reg.get_component<transformation>(t);
  tr.rotation + tr.rotation;
  auto &g = reg.get_component<gl_object>(t);

  systems::reset_gl_objects(g);

  return t;
}

gl_object get_cursor_geometry(const GLint program) {
  gl_object cursor;
  cursor.points = {{0.0f, 0.0f, 0.0f, 1.0f},  {1.0f, 0.0f, 0.0f, 1.0f},
                   {0.05f, 0.0f, 0.0f, 1.0f}, {1.0f, 0.0f, 0.0f, 1.0f},
                   {0.0f, 0.0f, 0.0f, 1.0f},  {0.0f, 1.0f, 0.0f, 1.0f},
                   {0.0f, 0.05f, 0.0f, 1.0f}, {0.0f, 1.0f, 0.0f, 1.0f},
                   {0.0f, 0.0f, 0.0f, 1.0f},  {0.0f, 0.0f, 1.0f, 1.0f},
                   {0.0f, 0.0f, 0.05f, 1.0f}, {0.0f, 0.0f, 1.0f, 1.0f}};

  cursor.indices = {0, 1, 2, 3, 4, 5};
  cursor.dmode = gl_object::draw_mode::lines;
  cursor.vtype = gl_object::vertex_t::point_color;
  cursor.program = program;

  return cursor;
}

ecs::EntityType add_icurve(const GLuint program) {
  std::vector<ecs::EntityType> sel_points;
  auto &reg = ecs::registry::get_registry();
  for (auto &[idx, _] : reg.get_map<selected>()) {
    if (reg.has_component<tag_point>(idx)) {
      sel_points.push_back(idx);
    }
  }

  if (sel_points.size() < 2)
    return ecs::null_entity;

  const auto b = reg.add_entity();
  reg.add_component<gl_object>(b, gl_object{program});
  reg.add_component<transformation>(b, {});
  reg.add_component<tag_figure>(
      b, tag_figure{"Cubic Interpolation Spline #" + std::to_string(b)});
  reg.add_component<icurve>(b, {});
  reg.add_component<tag_parent>(b, tag_parent{});
  reg.add_component<tag_visible>(b, {});

  auto &g = reg.get_component<gl_object>(b);
  g.color = g.primary;

  // add relationships to selected points
  for (const auto p : sel_points) {
    if (reg.has_component<relationship>(p)) {
      auto &r = reg.get_component<relationship>(p);
      r.parents.push_back(b);
    } else {
      reg.add_component<relationship>(p, {{b}, {}});
    }
  }

  reg.add_component<relationship>(b, {{}, std::move(sel_points)});
  systems::regenerate_icurve(b);
  g.dmode = gl_object::draw_mode::patches;
  g.patch_size = 4;

  return b;
}

ecs::EntityType add_bezier_surface(std::vector<ecs::EntityType> &points, unsigned int patches[2]) {
  static auto &reg = ecs::registry::get_registry();
  static auto &sm = shader_manager::get_manager();
  const auto builder = reg.add_entity();

  reg.add_component<bezier_surface_params>(builder, {});
  reg.add_component<gl_object>(builder, {});
  reg.add_component<relationship>(builder, {});
  reg.add_component<transformation>(builder, {});
  reg.add_component<tag_figure>(builder, {});
  reg.add_component<tag_parent>(builder, {});
  reg.add_component<tag_visible>(builder, {});

  auto &g = reg.get_component<gl_object>(builder);
  g.primary = {255.f / 255.f, 69.f / 255.f, 0.f, 1.0f};
  g.selected = {0.f, 255.f / 255.f, 171.f / 255.f, 1.0f};
  g.color = g.primary;
  g.program = sm.programs[shader_t::BEZIER_PATCH_SHADER].idx;

  auto &f = reg.get_component<tag_figure>(builder);
  f.name = "Bezier Surface #" + std::to_string(builder);

  auto &rel = reg.get_component<relationship>(builder);
  rel.indestructible_relation = true;

  const auto bezier_polygon = reg.add_entity();
  reg.add_component<gl_object>(
      bezier_polygon, gl_object{sm.programs[shader_t::GENERAL_SHADER].idx});
  reg.add_component<transformation>(bezier_polygon, {});
  auto &b_g = reg.get_component<gl_object>(bezier_polygon);
  b_g.dmode = gl_object::draw_mode::lines;

  auto &bsp = reg.get_component<bezier_surface_params>(builder);
  bsp.u = patches[0];
  bsp.v = patches[1];
  bsp.bezier_polygon = bezier_polygon;

  // 1. generate points as entities
  for (const auto point : points) {
    reg.add_component<relationship>(point, {});
    auto &prel = reg.get_component<relationship>(point);
    rel.children.push_back(point);
    prel.parents.push_back(builder);
    prel.indestructible_counter++;
  }

  // 3. regenerate all necessary buffers
  systems::regenerate_bezier_surface(builder);

  return builder;
}

ecs::EntityType add_bezier_surface(ecs::EntityType builder) {
  static auto &reg = ecs::registry::get_registry();
  static auto &sm = shader_manager::get_manager();

  auto &bsp = reg.get_component<bezier_surface_params>(builder);
  auto &g = reg.get_component<gl_object>(builder);
  g.program = sm.programs[shader_t::BEZIER_PATCH_SHADER].idx;
  auto &f = reg.get_component<tag_figure>(builder);
  f.name = "Bezier Surface #" + std::to_string(builder);
  reg.add_component<relationship>(builder, {});
  auto &rel = reg.get_component<relationship>(builder);
  rel.indestructible_relation = true;

  const auto bezier_polygon = reg.add_entity();
  reg.add_component<gl_object>(
      bezier_polygon, gl_object{sm.programs[shader_t::GENERAL_SHADER].idx});
  reg.add_component<transformation>(bezier_polygon, {});
  auto &b_g = reg.get_component<gl_object>(bezier_polygon);
  b_g.dmode = gl_object::draw_mode::lines;

  bsp.bezier_polygon = bezier_polygon;

  // 1. generate points as entities
  if (!bsp.cyllinder) {
    for (unsigned int j = 0; j <= 3 * bsp.v; ++j) {
      for (unsigned int i = 0; i <= 3 * bsp.u; ++i) {
        glm::vec3 pos;
        pos = bsp.root +
              glm::vec3(bsp.width / 3.f * i, 0.f, bsp.height / 3.f * j);
        auto point = add_point(transformation{pos},
                               sm.programs[shader_t::POINT_SHADER].idx);
        reg.add_component<relationship>(point, {});
        auto &prel = reg.get_component<relationship>(point);
        rel.children.push_back(point);
        prel.parents.push_back(builder);
        prel.indestructible_counter++;
      }
    }
  } else {
    const auto angle = (2 * glm::pi<float>()) / bsp.u;
    const auto dist =
        bsp.width * (4.f / 3.f) * tanf(glm::pi<float>() / (2.f * bsp.u));

    for (unsigned int j = 0; j <= 3 * bsp.v; ++j) {
      for (unsigned int i = 0; i < bsp.u; ++i) {
        // first rooted on the circle
        const auto tan1 =
            glm::normalize(glm::vec3{-sinf(i * angle), 0.0f, cosf(i * angle)});
        const auto tan2 = glm::normalize(
            glm::vec3{-sinf((i + 1) * angle), 0.f, cosf((i + 1) * angle)});

        const auto p1 = (bsp.root + glm::vec3{bsp.width * cosf(i * angle),
                                              j * bsp.height / 3.f,
                                              bsp.width * sinf(i * angle)});

        const auto p2 = p1 + dist * tan1;

        // second rooted on the circle
        const auto p4 =
            (bsp.root + glm::vec3{bsp.width * cosf((i + 1) * angle),
                                  j * bsp.height / 3.f,
                                  bsp.width * sinf((i + 1) * angle)});

        const auto p3 = p4 - dist * tan2;

        auto point1 = add_point(transformation{p1},
                                sm.programs[shader_t::POINT_SHADER].idx);
        reg.add_component<relationship>(point1, {});
        auto &prel1 = reg.get_component<relationship>(point1);
        rel.children.push_back(point1);
        prel1.parents.push_back(builder);
        prel1.indestructible_counter++;

        auto point2 = add_point(transformation{p2},
                                sm.programs[shader_t::POINT_SHADER].idx);
        reg.add_component<relationship>(point2, {});
        auto &prel2 = reg.get_component<relationship>(point2);
        rel.children.push_back(point2);
        prel2.parents.push_back(builder);
        prel2.indestructible_counter++;

        auto point3 = add_point(transformation{p3},
                                sm.programs[shader_t::POINT_SHADER].idx);
        reg.add_component<relationship>(point3, {});
        auto &prel3 = reg.get_component<relationship>(point3);
        rel.children.push_back(point3);
        prel3.parents.push_back(builder);
        prel3.indestructible_counter++;
      }
    }
  }
  // 2. delete builder tag
  reg.remove_component<tag_bezier_surface_builder>(builder);

  // 3. regenerate all necessary buffers
  systems::regenerate_bezier_surface(builder);

  return builder;
}

ecs::EntityType add_bezier_surface_builder(transformation &&_t,
                                           const GLuint program) {
  static auto &reg = ecs::registry::get_registry();

  const auto b = reg.add_entity();
  reg.add_component<gl_object>(b, gl_object{program});
  reg.add_component<transformation>(b, {});
  reg.add_component<tag_parent>(b, {});
  reg.add_component<tag_figure>(
      b, tag_figure{"[BUILDER] Bezier Surface #" + std::to_string(b)});
  reg.add_component<tag_bezier_surface_builder>(b, {});
  reg.add_component<bezier_surface_params>(b, {});
  reg.add_component<tag_visible>(b, {});

  auto &bsp = reg.get_component<bezier_surface_params>(b);
  bsp.root = _t.translation;

  auto &g = reg.get_component<gl_object>(b);
  g.color = g.primary;

  g.dmode = gl_object::draw_mode::lines;
  g.primary = {255.f / 255.f, 69.f / 255.f, 0.f, 1.0f};
  g.selected = {0.f, 255.f / 255.f, 171.f / 255.f, 1.0f};
  g.color = g.primary;

  systems::regenerate_bezier_surface_builder(b);

  return b;
}

ecs::EntityType add_bspline(const GLuint program) {
  auto &sm = shader_manager::get_manager();
  std::vector<ecs::EntityType> sel_points;
  auto &reg = ecs::registry::get_registry();
  for (auto &[idx, _] : reg.get_map<selected>()) {
    if (reg.has_component<tag_point>(idx)) {
      sel_points.push_back(idx);
    }
  }

  if (sel_points.size() < 1)
    return ecs::null_entity;

  const auto bezier_polygon = reg.add_entity();
  reg.add_component<gl_object>(bezier_polygon, gl_object{program});
  reg.add_component<transformation>(bezier_polygon, {});
  auto &b_g = reg.get_component<gl_object>(bezier_polygon);
  b_g.program = sm.programs[shader_t::GENERAL_SHADER].idx;
  b_g.dmode = gl_object::draw_mode::line_strip;

  const auto deboor_polygon = reg.add_entity();
  reg.add_component<gl_object>(deboor_polygon, gl_object{program});
  reg.add_component<transformation>(deboor_polygon, {});
  auto &db_g = reg.get_component<gl_object>(deboor_polygon);
  db_g.program = sm.programs[shader_t::GENERAL_SHADER].idx;
  db_g.dmode = gl_object::draw_mode::line_strip;

  const auto b = reg.add_entity();
  reg.add_component<gl_object>(b, gl_object{program});
  reg.add_component<transformation>(b, {});
  reg.add_component<tag_figure>(
      b, tag_figure{"B-Spline curve #" + std::to_string(b)});
  reg.add_component<bspline>(b, bspline{bezier_polygon, deboor_polygon});
  reg.add_component<tag_parent>(b, tag_parent{});
  reg.add_component<tag_visible>(b, {});

  auto &g = reg.get_component<gl_object>(b);

  // add relationships to selected points
  for (const auto p : sel_points) {
    if (reg.has_component<relationship>(p)) {
      auto &r = reg.get_component<relationship>(p);
      r.parents.push_back(b);
    } else {
      reg.add_component<relationship>(p, {{b}, {}});
    }
  }

  reg.add_component<relationship>(b, {{}, std::move(sel_points)});
  systems::regenerate_bspline(b);
  g.dmode = gl_object::draw_mode::patches;
  g.patch_size = 4;

  return b;
}

ecs::EntityType add_bezier(const GLuint program) {
  auto &reg = ecs::registry::get_registry();
  auto &sm = shader_manager::get_manager();
  std::vector<ecs::EntityType> sel_points;
  for (auto &[idx, _] : reg.get_map<selected>()) {
    if (reg.has_component<tag_point>(idx)) {
      sel_points.push_back(idx);
    }
  }

  if (sel_points.size() < 1)
    return ecs::null_entity;

  const auto bezier_polygon = reg.add_entity();
  reg.add_component<gl_object>(bezier_polygon, gl_object{program});
  reg.add_component<transformation>(bezier_polygon, {});
  auto &sec_g = reg.get_component<gl_object>(bezier_polygon);
  sec_g.dmode = gl_object::draw_mode::line_strip;
  sec_g.program = sm.programs[shader_t::GENERAL_SHADER].idx;

  const auto b = reg.add_entity();
  reg.add_component<gl_object>(b, gl_object{program});
  reg.add_component<transformation>(b, {});
  reg.add_component<tag_figure>(
      b, tag_figure{"bezier curve #" + std::to_string(b)});
  reg.add_component<bezierc>(b, bezierc{bezier_polygon});
  reg.add_component<tag_parent>(b, tag_parent{});
  reg.add_component<tag_visible>(b, {});

  auto &g = reg.get_component<gl_object>(b);

  // add relationships to selected points
  for (const auto p : sel_points) {
    if (reg.has_component<relationship>(p)) {
      auto &r = reg.get_component<relationship>(p);
      r.parents.push_back(b);
    } else {
      reg.add_component<relationship>(p, {{b}, {}});
    }
  }

  reg.add_component<relationship>(b, {{}, std::move(sel_points)});
  systems::regenerate_bezier(b);
  g.dmode = gl_object::draw_mode::patches;
  g.patch_size = 4;

  return b;
}

ecs::EntityType add_point(transformation &&_t, const GLuint program) {
  auto &reg = ecs::registry::get_registry();
  const auto p = reg.add_entity();
  reg.add_component<transformation>(p, std::move(_t));
  reg.add_component<gl_object>(p, gl_object{program});
  reg.add_component<tag_point>(p, tag_point{});

  reg.add_component<tag_visible>(p, {});
  reg.add_component<tag_clickable>(p, {});
  reg.add_component<tag_figure>(p, tag_figure{"point #" + std::to_string(p)});

  auto &g = reg.get_component<gl_object>(p);

  g.points.push_back({0.0f, 0.0f, 0.0f, 1.0f});
  g.indices.push_back(0u);
  g.dmode = gl_object::draw_mode::points;

  systems::reset_gl_objects(g);

  return p;
}

ecs::EntityType add_virtual_point(transformation &&_t, const GLuint program) {
  auto &reg = ecs::registry::get_registry();
  const auto p = reg.add_entity();
  reg.add_component<transformation>(p, std::move(_t));
  reg.add_component<gl_object>(p, gl_object{program});
  reg.add_component<tag_point>(p, tag_point{});
  reg.add_component<tag_virtual>(p, tag_virtual{});
  reg.add_component<tag_clickable>(p, {});
  reg.add_component<relationship>(p, {});

  auto &g = reg.get_component<gl_object>(p);

  g.points.push_back({0.0f, 0.0f, 0.0f, 1.0f});
  g.indices.push_back(0u);
  g.dmode = gl_object::draw_mode::points;
  g.color = {0.0f, 1.0f, 0.0f, 1.0f};
  g.primary = {0.0f, 1.0f, 0.0f, 1.0f};
  g.selected = {1.0f, 1.0f, 0.0f, 1.0f};

  systems::reset_gl_objects(g);

  return p;
}

ecs::EntityType add_center_of_weight(transformation &&_t,
                                     const GLuint program) {
  auto &reg = ecs::registry::get_registry();
  const auto p = reg.add_entity();
  reg.add_component<transformation>(p, std::move(_t));
  reg.add_component<gl_object>(p, gl_object{program});
  reg.add_component<tag_point>(p, tag_point{});
  reg.add_component<tag_center_of_weight>(p, {});

  auto &g = reg.get_component<gl_object>(p);

  g.points.push_back({0.0f, 0.0f, 0.0f, 1.0f});
  g.indices.push_back(0u);
  g.dmode = gl_object::draw_mode::points;
  g.primary = {1.0f, 1.0f, 1.0f, 1.0f};
  g.color = {1.0f, 1.0f, 1.0f, 1.0f};

  systems::reset_gl_objects(g);

  return p;
}

ecs::EntityType add_torus(parametric &&_p, transformation &&_t,
                          torus_params &&_tp, const GLuint program) {
  auto &reg = ecs::registry::get_registry();

  const auto t = reg.add_entity();
  reg.add_component<parametric>(t, std::move(_p));
  reg.add_component<transformation>(t, std::move(_t));
  reg.add_component<gl_object>(t, gl_object{program});
  reg.add_component<torus_params>(t, std::move(_tp));
  reg.add_component<tag_figure>(t, tag_figure{"torus #" + std::to_string(t)});
  reg.add_component<tag_visible>(t, {});
  reg.add_component<tag_clickable>(t, {});

  auto &g = reg.get_component<gl_object>(t);
  auto &tp = reg.get_component<torus_params>(t);
  auto &p = reg.get_component<parametric>(t);

  systems::generate_torus_points(tp, p, g.points);
  systems::generate_torus_lines(p, g.points, g.indices);
  systems::reset_gl_objects(g);

  return t;
}

void setup_initial_geometry() {
  auto &sm = shader_manager::get_manager();
  add_cursor(transformation{},
             get_cursor_geometry(sm.programs[shader_t::CURSOR_SHADER].idx));
  add_center_of_weight({}, sm.programs[shader_t::POINT_SHADER].idx);
}

ecs::EntityType add_bspline_surface(std::vector<ecs::EntityType> &points, unsigned int patches[2]) {
  static auto &reg = ecs::registry::get_registry();
  static auto &sm = shader_manager::get_manager();

  const auto builder = reg.add_entity();
  reg.add_component<gl_object>(builder, gl_object{});
  reg.add_component<transformation>(builder, {});
  reg.add_component<tag_parent>(builder, {});
  reg.add_component<tag_figure>(
      builder,
      tag_figure{"[BUILDER] B-Spline Surface #" + std::to_string(builder)});
  reg.add_component<bspline_surface_params>(builder, {});
  reg.add_component<tag_visible>(builder, {});
  reg.add_component<relationship>(builder, {});

  auto &bsp = reg.get_component<bspline_surface_params>(builder);
  bsp.u = patches[0];
  bsp.v = patches[1];

  auto &g = reg.get_component<gl_object>(builder);
  g.color = g.primary;
  g.dmode = gl_object::draw_mode::lines;
  g.primary = {255.f / 255.f, 69.f / 255.f, 0.f, 1.0f};
  g.selected = {0.f, 255.f / 255.f, 171.f / 255.f, 1.0f};
  g.color = g.primary;
  g.program = sm.programs[shader_t::BSPLINE_PATCH_SHADER].idx;

  auto &f = reg.get_component<tag_figure>(builder);
  f.name = "B-Spline Surface #" + std::to_string(builder);

  auto &rel = reg.get_component<relationship>(builder);
  rel.indestructible_relation = true;

  const auto deboor_polygon = reg.add_entity();
  reg.add_component<gl_object>(
      deboor_polygon, gl_object{sm.programs[shader_t::GENERAL_SHADER].idx});
  reg.add_component<transformation>(deboor_polygon, {});
  auto &b_g = reg.get_component<gl_object>(deboor_polygon);
  b_g.dmode = gl_object::draw_mode::lines;

  bsp.deboor_polygon = deboor_polygon;

  // 1. generate points as entities
  for (auto &point : points) {
    reg.add_component<relationship>(point, {});
    auto &prel = reg.get_component<relationship>(point);
    rel.children.push_back(point);
    prel.parents.push_back(builder);
    prel.indestructible_counter++;
  }

  // 2. regenerate all necessary buffers
  systems::regenerate_bspline_surface(builder);

  return builder;
}

ecs::EntityType add_bspline_surface(ecs::EntityType builder) {
  static auto &reg = ecs::registry::get_registry();
  static auto &sm = shader_manager::get_manager();

  auto &bsp = reg.get_component<bspline_surface_params>(builder);
  auto &g = reg.get_component<gl_object>(builder);
  g.program = sm.programs[shader_t::BSPLINE_PATCH_SHADER].idx;
  auto &f = reg.get_component<tag_figure>(builder);
  f.name = "B-Spline Surface #" + std::to_string(builder);
  reg.add_component<relationship>(builder, {});
  auto &rel = reg.get_component<relationship>(builder);
  rel.indestructible_relation = true;

  const auto deboor_polygon = reg.add_entity();
  reg.add_component<gl_object>(
      deboor_polygon, gl_object{sm.programs[shader_t::GENERAL_SHADER].idx});
  reg.add_component<transformation>(deboor_polygon, {});
  auto &b_g = reg.get_component<gl_object>(deboor_polygon);
  b_g.dmode = gl_object::draw_mode::lines;

  bsp.deboor_polygon = deboor_polygon;

  // 1. generate points as entities
  if (!bsp.cyllinder) {
    for (unsigned int j = 0; j < 4 + bsp.v - 1; ++j) {
      for (unsigned int i = 0; i < 4 + bsp.u - 1; ++i) {
        glm::vec3 pos;
        pos = bsp.root +
              glm::vec3(bsp.width / 3.f * i, 0.f, bsp.height / 3.f * j);
        auto point = add_point(transformation{pos},
                               sm.programs[shader_t::POINT_SHADER].idx);
        reg.add_component<relationship>(point, {});
        auto &prel = reg.get_component<relationship>(point);
        rel.children.push_back(point);
        prel.parents.push_back(builder);
        prel.indestructible_counter++;
      }
    }
  } else {
    const auto angle = (2 * glm::pi<float>()) / (4 + bsp.u - 1 - 3);
    for (unsigned int j = 0; j < (4 + bsp.v - 1); ++j) {
      for (unsigned int i = 0; i < (4 + bsp.u - 1 - 3); ++i) {
        const auto p1 = (bsp.root + glm::vec3{bsp.width * cosf(i * angle),
                                              j * bsp.height / 3.f,
                                              bsp.width * sinf(i * angle)});

        auto point1 = add_point(transformation{p1},
                                sm.programs[shader_t::POINT_SHADER].idx);
        reg.add_component<relationship>(point1, {});
        auto &prel1 = reg.get_component<relationship>(point1);
        rel.children.push_back(point1);
        prel1.parents.push_back(builder);
        prel1.indestructible_counter++;
      }
    }
  }
  // 2. delete builder tag
  reg.remove_component<tag_bspline_surface_builder>(builder);

  // 3. regenerate all necessary buffers
  systems::regenerate_bspline_surface(builder);

  return builder;
}

ecs::EntityType add_bspline_surface_builder(transformation &&_t,
                                            const GLuint program) {
  static auto &reg = ecs::registry::get_registry();

  const auto b = reg.add_entity();
  reg.add_component<gl_object>(b, gl_object{program});
  reg.add_component<transformation>(b, {});
  reg.add_component<tag_parent>(b, {});
  reg.add_component<tag_figure>(
      b, tag_figure{"[BUILDER] B-Spline Surface #" + std::to_string(b)});
  reg.add_component<tag_bspline_surface_builder>(b, {});
  reg.add_component<bspline_surface_params>(b, {});
  reg.add_component<tag_visible>(b, {});

  auto &bsp = reg.get_component<bspline_surface_params>(b);
  bsp.root = _t.translation;

  auto &g = reg.get_component<gl_object>(b);
  g.color = g.primary;

  g.dmode = gl_object::draw_mode::lines;
  g.primary = {255.f / 255.f, 69.f / 255.f, 0.f, 1.0f};
  g.selected = {0.f, 255.f / 255.f, 171.f / 255.f, 1.0f};
  g.color = g.primary;

  systems::regenerate_bspline_surface_builder(b);

  return b;
}
} // namespace constructors
