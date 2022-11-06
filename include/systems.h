#pragma once

#include <ecs.h>
#include <gl_object.h>
#include <input_state.h>
#include <memory>
#include <parametric.h>
#include <registry.h>
#include <sampler.h>
#include <torus.h>
#include <vector>

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

namespace systems {

void render_gl(const gl_object &g, bool only_bspline = false);

void render_visible_entities(bool only_bspline = false);

void render_app(bool only_bspline = false);

void get_gizmo_transform(glm::mat4 &gtrans);

void apply_group_transform(glm::mat4 &gtrans);

void refresh_common_uniforms(GLuint program);

glm::vec4 sample_torus(const torus_params &tp, const float u, const float v);

bool generate_torus_points(const torus_params &s, const parametric &p,
                           std::vector<glm::vec4> &out_vertices);

void regenerate_bezier(ecs::EntityType idx);
void regenerate_gregory(ecs::EntityType idx);
void regenerate_bezier_surface_builder(ecs::EntityType idx);
void regenerate_bezier_surface(ecs::EntityType idx);

void regenerate_bspline(ecs::EntityType idx);
void regenerate_bspline_surface_builder(ecs::EntityType idx);
void regenerate_bspline_surface(ecs::EntityType idx);

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

enum class intersection_status {
  success,
  self_intersection_error,
  start_point_gradient_edge_error,
  start_point_gradient_final_point_error,
  start_point_subdivisions_final_point_error,
  start_point_gradient_error,
  newton_edge_error,
  nan_error,
  other_error,
  already_has_intersection_error
};

struct intersection_params {
  int subdivisions = 8;
  int subdivisions_iterations = 10;
  int gradient_iters = 1000;
  float start_delta = 1e-4f;
  float start_acceptance = 1e-2f;
  float subdivisions_acceptance = 1e-1f;

  int newton_iters = 100;
  float newton_acceptance = 1e-2;
  float cycle_acceptance = 1e-2;
  float delta = 1e-2;

  bool start_from_cursor{false};
  float cursor_dist = 0.1f;
};

intersection_status
intersect(ecs::EntityType first_idx, ecs::EntityType second_idx, sampler &first,
          sampler &second, const intersection_params &params,
          bool self_intersection, const glm::vec3 &cursor_pos);

} // namespace systems
