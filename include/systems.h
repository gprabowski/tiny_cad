#pragma once

#include "torus.h"
#include <app_state.h>
#include <ecs.h>
#include <gl_object.h>
#include <memory>
#include <parametric.h>
#include <registry.h>
#include <vector>

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

namespace systems {

void render_points(const gl_object &g);

void render_figures(
    const std::vector<ecs::EntityType> &selected_parents,
    const std::vector<ecs::EntityType> &selected_primitives,
    const std::vector<ecs::EntityType> &unselected_indices,
    ecs::ComponentStorage<transformation> &transformation_component,
    ecs::ComponentStorage<gl_object> &ogl_component,
    std::shared_ptr<GLFWwindow> w, std::shared_ptr<app_state> s,
    glm::vec3 &center_out);

void render_app(ecs::registry &reg, std::shared_ptr<app_state> s,
                std::vector<ecs::EntityType> &sel,
                std::vector<ecs::EntityType> &unsel,
                std::vector<ecs::EntityType> &changed);

void get_gizmo_transform(
    glm::mat4 &gtrans, std::vector<ecs::EntityType> &indices,
    ecs::ComponentStorage<transformation> &transformation_component,
    std::shared_ptr<app_state> &s, const glm::vec3 &center);

void apply_group_transform(
    glm::mat4 &gtrans, std::vector<ecs::EntityType> &indices,
    ecs::ComponentStorage<transformation> &transformation_component,
    std::vector<ecs::EntityType> &changed, const glm::vec3 &center);

void render_cursors(
    const std::vector<ecs::EntityType> indices,
    const ecs::ComponentStorage<gl_object> &ogl_components,
    ecs::ComponentStorage<transformation> &transformation_component,
    std::shared_ptr<GLFWwindow> w, std::shared_ptr<app_state> s);

void refresh_common_uniforms(GLuint program);

glm::vec4 sample_torus(const torus_params &tp, const float u, const float v);

bool generate_torus_points(const torus_params &s, const parametric &p,
                           std::vector<glm::vec4> &out_vertices);

bool regenerate_bezier(const relationship &r, adaptive &a,
                       ecs::ComponentStorage<transformation> transformations,
                       ecs::ComponentStorage<relationship> relationships,
                       std::vector<glm::vec4> &out_vertices,
                       std::vector<unsigned int> &out_indices,
                       std::vector<glm::vec4> &out_vertices_polygon,
                       std::vector<unsigned int> &out_indices_polygon);

bool regenerate_bspline(relationship &r, adaptive &a,
                        ecs::ComponentStorage<transformation> transformations,
                        ecs::ComponentStorage<relationship> relationships,
                        std::vector<glm::vec4> &out_vertices,
                        std::vector<unsigned int> &out_indices,
                        std::vector<glm::vec4> &out_vertices_polygon,
                        std::vector<unsigned int> &out_indices_polygon);

void add_sel_points_to_parent(ecs::EntityType idx, ecs::registry &reg);

void generate_torus_lines(const parametric &p,
                          const std::vector<glm::vec4> &points,
                          std::vector<unsigned int> &indices);

void reset_gl_objects(gl_object &g);
void set_model_uniform(const transformation &t);
void update_changed_relationships(ecs::registry &reg,
                                  std::shared_ptr<app_state> &s,
                                  const std::vector<ecs::EntityType> &changed,
                                  const std::vector<ecs::EntityType> &del);
void delete_entities(ecs::registry &reg,
                     const std::vector<ecs::EntityType> &del);

} // namespace systems
