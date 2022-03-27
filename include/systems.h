#pragma once

#include "torus.h"
#include <app_state.h>
#include <component_manager.h>
#include <ecs.h>
#include <gl_object.h>
#include <memory>
#include <parametric.h>
#include <vector>

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

namespace systems {

void render_points(const gl_object &g);

void render_figures(
    const std::vector<ecs::EntityType> &selected_indices,
    const std::vector<ecs::EntityType> &unselected_indices,
    ecs::ComponentStorage<transformation> &transformation_component,
    ecs::ComponentStorage<gl_object> &ogl_component,
    std::shared_ptr<GLFWwindow> w, std::shared_ptr<app_state> s,
    glm::vec3 &center_out);

void render_app(ecs::component_manager &cm, std::shared_ptr<GLFWwindow> &w,
                std::shared_ptr<app_state> s, std::vector<ecs::EntityType> &sel,
                std::vector<ecs::EntityType> &unsel);

bool render_and_apply_gizmo(
    std::vector<ecs::EntityType> &indices,
    ecs::ComponentStorage<transformation> &transformation_component,
    std::shared_ptr<GLFWwindow> &w, std::shared_ptr<app_state> &s,
    const glm::vec3 &center);

void render_cursors(
    const std::vector<ecs::EntityType> indices,
    const ecs::ComponentStorage<gl_object> &ogl_components,
    ecs::ComponentStorage<transformation> &transformation_component,
    std::shared_ptr<GLFWwindow> w, std::shared_ptr<app_state> s);

void refresh_common_uniforms(GLuint program, const glm::mat4 &view,
                             const glm::mat4 proj,
                             std::shared_ptr<GLFWwindow> w,
                             std::shared_ptr<app_state> s);

glm::vec4 sample_torus(const torus_params &tp, const float u, const float v);

template <typename ParamsType>
inline glm::vec4 sample(const ParamsType &p, const float u, const float v) {
  if constexpr (std::is_same_v<ParamsType, torus_params>) {
    return sample_torus(p, u, v);
  }
  return glm::vec4();
}

template <typename ParamsType>
bool generate_points(const ParamsType &s, const parametric &p,
                     std::vector<glm::vec4> &out_vertices) {

  const auto u_diff = p.u_max - p.u_min;
  const auto v_diff = p.v_max - p.v_min;

  const auto u_inv_div = 1.0f / static_cast<float>(p.samples[0]);
  const auto v_inv_div = 1.0f / static_cast<float>(p.samples[1]);

  for (int i = 0u; i < p.samples[0]; ++i) {
    for (int j = 0u; j < p.samples[1]; ++j) {
      out_vertices.emplace_back(
          sample<ParamsType>(s, p.u_min + u_diff * i * u_inv_div,
                             p.v_min + v_diff * j * v_inv_div));
    }
  }
  return true;
}

bool regenerate_bezier(const relationship &r,
                       ecs::ComponentStorage<transformation> transformations,
                       ecs::ComponentStorage<relationship> relationships,
                       std::vector<glm::vec4> &out_vertices,
                       std::vector<unsigned int> &out_indices);

void generate_lines(const parametric &p, const std::vector<glm::vec4> &points,
                    std::vector<unsigned int> &indices);

void reset_gl_objects(gl_object &g);
void set_model_uniform(const transformation &t);
void update_changed_relationships(ecs::component_manager &cm,
                                  const std::vector<ecs::EntityType> &sel,
                                  const std::vector<ecs::EntityType> &del);
void delete_entities(ecs::component_manager &cm,
                     const std::vector<ecs::EntityType> &del);

} // namespace systems
