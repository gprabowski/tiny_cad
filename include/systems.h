#pragma once

#include "torus.h"
#include <ecs.h>
#include <gl_object.h>
#include <input_state.h>
#include <memory>
#include <parametric.h>
#include <registry.h>
#include <vector>

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

namespace systems {

void render_gl(const gl_object &g);

void render_visible_entities();

void render_app();

void get_gizmo_transform(glm::mat4 &gtrans);

void apply_group_transform(glm::mat4 &gtrans);

void refresh_common_uniforms(GLuint program);

glm::vec4 sample_torus(const torus_params &tp, const float u, const float v);

bool generate_torus_points(const torus_params &s, const parametric &p,
                           std::vector<glm::vec4> &out_vertices);

void regenerate_bezier(ecs::EntityType idx);

void regenerate_bspline(ecs::EntityType idx);

void regenerate_icurve(ecs::EntityType idx);

void add_sel_points_to_parent(ecs::EntityType idx);

void generate_torus_lines(const parametric &p,
                          const std::vector<glm::vec4> &points,
                          std::vector<unsigned int> &indices);

void reset_gl_objects(gl_object &g);
void set_model_uniform(const transformation &t);
void update_changed_relationships();
void delete_entities();

void regenerate(ecs::EntityType idx);

} // namespace systems
