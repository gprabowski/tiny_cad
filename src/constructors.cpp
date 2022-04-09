#include <constructors.h>
#include <log.h>
#include <relationship.h>
#include <systems.h>

namespace constructors {

ecs::EntityType add_cursor(ecs::registry &reg, transformation &&_t,
                           gl_object &&_g) {
  const auto t = reg.add_entity();
  reg.add_component<transformation>(t, std::move(_t));
  reg.add_component<gl_object>(t, std::move(_g));
  reg.add_component<cursor_params>(t, cursor_params{});

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

ecs::EntityType add_bspline(ecs::registry &reg, std::shared_ptr<app_state> &s,
                            const GLuint program) {
  std::vector<ecs::EntityType> sel_points;
  for (auto &[idx, _] : reg.get_map<selected>()) {
    if (reg.has_component<tag_point>(idx)) {
      sel_points.push_back(idx);
    }
  }

  if (sel_points.size() < 1)
    return ecs::null_entity;

  const auto secondary = reg.add_entity();
  reg.add_component<gl_object>(secondary, gl_object{program});
  auto &sec_g = reg.get_component<gl_object>(secondary);
  sec_g.dmode = gl_object::draw_mode::line_strip;

  const auto b = reg.add_entity();
  reg.add_component<gl_object>(b, gl_object{program});
  reg.add_component<transformation>(b, {});
  reg.add_component<tag_figure>(
      b, tag_figure{"B-Spline curve #" + std::to_string(b)});
  reg.add_component<tag_bspline>(b, tag_bspline{});
  reg.add_component<tag_parent>(b, tag_parent{});
  reg.add_component<secondary_object>(b, {secondary});
  reg.add_component<adaptive>(b, {});

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
  auto &r = reg.get_component<relationship>(b);
  auto &a = reg.get_component<adaptive>(b);

  systems::regenerate_bspline(r, a, reg.get_map<transformation>(),
                              reg.get_map<relationship>(), g.points, g.indices,
                              sec_g.points, sec_g.indices);

  g.dmode = gl_object::draw_mode::line_strip;
  systems::reset_gl_objects(g);
  systems::reset_gl_objects(sec_g);

  return b;
}

ecs::EntityType add_bezier(ecs::registry &reg, std::shared_ptr<app_state> &s,
                           const GLuint program) {
  std::vector<ecs::EntityType> sel_points;
  for (auto &[idx, _] : reg.get_map<selected>()) {
    if (reg.has_component<tag_point>(idx)) {
      sel_points.push_back(idx);
    }
  }

  if (sel_points.size() < 1)
    return ecs::null_entity;

  const auto secondary = reg.add_entity();
  reg.add_component<gl_object>(secondary, gl_object{program});
  auto &sec_g = reg.get_component<gl_object>(secondary);
  sec_g.dmode = gl_object::draw_mode::line_strip;

  const auto b = reg.add_entity();
  reg.add_component<gl_object>(b, gl_object{program});
  reg.add_component<transformation>(b, {});
  reg.add_component<tag_figure>(
      b, tag_figure{"bezier curve #" + std::to_string(b)});
  reg.add_component<tag_bezierc>(b, tag_bezierc{});
  reg.add_component<tag_parent>(b, tag_parent{});
  reg.add_component<secondary_object>(b, {secondary});
  reg.add_component<adaptive>(b, {});

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
  auto &r = reg.get_component<relationship>(b);
  auto &a = reg.get_component<adaptive>(b);

  systems::regenerate_bezier(r, a, reg.get_map<transformation>(),
                             reg.get_map<relationship>(), g.points, g.indices,
                             sec_g.points, sec_g.indices);

  g.dmode = gl_object::draw_mode::line_strip;
  systems::reset_gl_objects(g);
  systems::reset_gl_objects(sec_g);

  return b;
}

ecs::EntityType add_point(ecs::registry &reg, transformation &&_t,
                          const GLuint program) {
  const auto p = reg.add_entity();
  reg.add_component<transformation>(p, std::move(_t));
  reg.add_component<gl_object>(p, gl_object{program});
  reg.add_component<tag_figure>(p, tag_figure{"point #" + std::to_string(p)});
  reg.add_component<tag_point>(p, tag_point{});

  auto &g = reg.get_component<gl_object>(p);

  g.points.push_back({0.0f, 0.0f, 0.0f, 1.0f});
  g.indices.push_back(0u);
  g.dmode = gl_object::draw_mode::points;

  systems::reset_gl_objects(g);

  return p;
}

ecs::EntityType add_torus(ecs::registry &reg, parametric &&_p,
                          transformation &&_t, torus_params &&_tp,
                          const GLuint program) {
  const auto t = reg.add_entity();
  reg.add_component<parametric>(t, std::move(_p));
  reg.add_component<transformation>(t, std::move(_t));
  reg.add_component<gl_object>(t, gl_object{program});
  reg.add_component<torus_params>(t, std::move(_tp));
  reg.add_component<tag_figure>(t, tag_figure{"torus #" + std::to_string(t)});

  auto &g = reg.get_component<gl_object>(t);
  auto &tp = reg.get_component<torus_params>(t);
  auto &p = reg.get_component<parametric>(t);

  systems::generate_torus_points(tp, p, g.points);
  systems::generate_torus_lines(p, g.points, g.indices);
  systems::reset_gl_objects(g);

  return t;
}

void setup_initial_geometry(ecs::registry &reg, GLuint program) {
  add_cursor(reg, transformation{}, get_cursor_geometry(program));
}
} // namespace constructors
