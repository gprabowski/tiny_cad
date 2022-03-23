#include <constructors.h>
#include <systems.h>

namespace constructors {

ecs::EntityType add_cursor(ecs::component_manager &cm, transformation &&_t,
                           gl_object &&_g) {
  const auto t = cm.add_entity();
  cm.add_component<transformation>(t, std::move(_t));
  cm.add_component<gl_object>(t, std::move(_g));
  cm.add_component<cursor_params>(t, cursor_params{});

  auto &g = cm.get_component<gl_object>(t);

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

ecs::EntityType add_point(ecs::component_manager &cm, transformation &&_t,
                          const GLuint program) {
  const auto p = cm.add_entity();
  cm.add_component<transformation>(p, std::move(_t));
  cm.add_component<gl_object>(p, gl_object{program});
  cm.add_component<tag_figure>(p, tag_figure{"point #" + std::to_string(p)});
  cm.add_component<tag_point>(p, tag_point{});

  auto &g = cm.get_component<gl_object>(p);

  g.points.push_back({0.0f, 0.0f, 0.0f, 1.0f});
  g.indices.push_back(0u);
  g.dmode = gl_object::draw_mode::points;

  systems::reset_gl_objects(g);

  return p;
}

ecs::EntityType add_torus(ecs::component_manager &cm, parametric &&_p,
                          transformation &&_t, torus_params &&_tp,
                          const GLuint program) {
  const auto t = cm.add_entity();
  cm.add_component<parametric>(t, std::move(_p));
  cm.add_component<transformation>(t, std::move(_t));
  cm.add_component<gl_object>(t, gl_object{program});
  cm.add_component<torus_params>(t, std::move(_tp));
  cm.add_component<tag_figure>(t, tag_figure{"torus #" + std::to_string(t)});

  auto &g = cm.get_component<gl_object>(t);
  auto &tp = cm.get_component<torus_params>(t);
  auto &p = cm.get_component<parametric>(t);

  systems::generate_points(tp, p, g.points);
  systems::generate_lines(p, g.points, g.indices);
  systems::reset_gl_objects(g);

  return t;
}
} // namespace constructors
