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

  return b;
}
ecs::EntityType add_bspline(const GLuint program) {
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
  b_g.dmode = gl_object::draw_mode::line_strip;

  const auto deboor_polygon = reg.add_entity();
  reg.add_component<gl_object>(deboor_polygon, gl_object{program});
  reg.add_component<transformation>(deboor_polygon, {});
  auto &db_g = reg.get_component<gl_object>(deboor_polygon);
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

  return b;
}

ecs::EntityType add_bezier(const GLuint program) {
  auto &reg = ecs::registry::get_registry();
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
} // namespace constructors
