#pragma once

#include <memory>

#include <glad/glad.h>

#include <input_state.h>
#include <registry.h>

namespace constructors {
gl_object get_cursor_geometry(const GLint program);
ecs::EntityType add_cursor(transformation &&_t, gl_object &&_g);

void setup_initial_geometry();

ecs::EntityType add_point(transformation &&_t, const GLuint program);

ecs::EntityType add_virtual_point(transformation &&_t, const GLuint program);

ecs::EntityType add_center_of_weight(transformation &&_t, const GLuint program);

ecs::EntityType add_torus(parametric &&_p, transformation &&_t,
                          torus_params &&_tp, const GLuint program);

ecs::EntityType add_bezier(const GLuint program);

ecs::EntityType add_bspline(const GLuint program);

ecs::EntityType add_icurve(const GLuint program);
} // namespace constructors
