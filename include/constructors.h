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
ecs::EntityType add_bezier_impl(const GLuint program,
                                const std::vector<ecs::EntityType> &points);

ecs::EntityType add_bspline(const GLuint program);
ecs::EntityType add_bspline_impl(const GLuint program,
                                 const std::vector<ecs::EntityType> &points);

ecs::EntityType add_icurve(const GLuint program);
ecs::EntityType add_icurve(std::vector<glm::vec4>& points, const GLuint program);
ecs::EntityType add_icurve_impl(const GLuint program,
                                const std::vector<ecs::EntityType> &points);

ecs::EntityType add_gregory(const GLuint program);

ecs::EntityType add_bezier_surface_builder(transformation &&_t,
                                           const GLuint program);

ecs::EntityType add_bezier_surface(ecs::EntityType builder);

ecs::EntityType add_bspline_surface_builder(transformation &&_t,
                                            const GLuint program);

ecs::EntityType add_bspline_surface(ecs::EntityType builder);

ecs::EntityType add_intersection(std::vector<glm::vec3>& points);

ecs::EntityType add_bezier_surface(std::vector<ecs::EntityType> &points,
                                   unsigned int patches[2], bool cyllinder,
                                   glm::vec2 &samples);

ecs::EntityType add_bspline_surface(std::vector<ecs::EntityType> &points,
                                    unsigned int patches[2], bool cyllinder,
                                    glm::vec2 &samples);
} // namespace constructors
