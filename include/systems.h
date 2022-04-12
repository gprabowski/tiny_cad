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

void render_points(const gl_object &g);

void render_visible_entities();

void render_app(std::vector<ecs::EntityType> &changed);

void get_gizmo_transform(glm::mat4 &gtrans);

void apply_group_transform(glm::mat4 &gtrans,
                           std::vector<ecs::EntityType> &changed);

void refresh_common_uniforms(GLuint program);

glm::vec4 sample_torus(const torus_params &tp, const float u, const float v);

bool generate_torus_points(const torus_params &s, const parametric &p,
                           std::vector<glm::vec4> &out_vertices);

bool regenerate_bezier(const relationship &r, adaptive &a,
                       std::vector<glm::vec4> &out_vertices,
                       std::vector<unsigned int> &out_indices,
                       std::vector<glm::vec4> &out_vertices_polygon,
                       std::vector<unsigned int> &out_indices_polygon);

bool regenerate_bspline(ecs::EntityType idx, relationship &r, adaptive &a,
                        std::vector<glm::vec4> &out_vertices,
                        std::vector<unsigned int> &out_indices,
                        std::vector<glm::vec4> &out_vertices_polygon,
                        std::vector<unsigned int> &out_indices_polygon);

void add_sel_points_to_parent(ecs::EntityType idx);

void generate_torus_lines(const parametric &p,
                          const std::vector<glm::vec4> &points,
                          std::vector<unsigned int> &indices);

void reset_gl_objects(gl_object &g);
void set_model_uniform(const transformation &t);
void update_changed_relationships(const std::vector<ecs::EntityType> &changed,
                                  const std::vector<ecs::EntityType> &del);
void delete_entities(const std::vector<ecs::EntityType> &del);

} // namespace systems
