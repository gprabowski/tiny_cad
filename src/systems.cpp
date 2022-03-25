#include "gl_object.h"
#include "torus.h"
#include <glad/glad.h>

#define GLM_FORCE_RADIANS
#include <glm/ext/matrix_transform.hpp>
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>

#include <systems.h>

namespace systems {

void reset_gl_objects(gl_object &g) {
  if (!glIsBuffer(g.vbo)) {
    glCreateBuffers(1, &g.vbo);
  }
  // allocation or reallocation
  glNamedBufferData(g.vbo, sizeof(g.points[0]) * g.points.size(),
                    g.points.data(), GL_DYNAMIC_DRAW);

  if (!glIsBuffer(g.ebo)) {
    glCreateBuffers(1, &g.ebo);
  }
  // allocation or reallocation
  glNamedBufferData(g.ebo, sizeof(unsigned int) * g.indices.size(),
                    g.indices.data(), GL_DYNAMIC_DRAW);

  if (!glIsVertexArray(g.vao)) {
    glCreateVertexArrays(1, &g.vao);
  }

  glVertexArrayVertexBuffer(g.vao, 0, g.vbo, 0, sizeof(g.points[0]));
  glVertexArrayElementBuffer(g.vao, g.ebo);

  switch (g.vtype) {
  case gl_object::vertex_t::point: {
    glBindVertexArray(g.vao);
    glBindVertexBuffer(0, g.vbo, 0, sizeof(glm::vec4));
    glEnableVertexArrayAttrib(g.vao, 0);
    glVertexArrayAttribFormat(g.vao, 0, 4, GL_FLOAT, GL_FALSE, 0);
    glVertexArrayAttribBinding(g.vao, 0, 0);
    glBindVertexArray(0);
  } break;
  case gl_object::vertex_t::point_color: {
    glBindVertexArray(g.vao);
    glBindVertexBuffer(0, g.vbo, 0, 2 * sizeof(glm::vec4));

    glEnableVertexArrayAttrib(g.vao, 0);
    glEnableVertexArrayAttrib(g.vao, 1);

    glVertexArrayAttribFormat(g.vao, 0, 4, GL_FLOAT, GL_FALSE, 0);
    glVertexArrayAttribFormat(g.vao, 1, 4, GL_FLOAT, GL_FALSE,
                              sizeof(glm::vec4));

    glVertexArrayAttribBinding(g.vao, 0, 0);
    glVertexArrayAttribBinding(g.vao, 1, 0);
    glBindVertexArray(0);
  } break;

  default:
    break;
  }
}

void set_model_uniform(const transformation &t) {
  GLint program;
  glGetIntegerv(GL_CURRENT_PROGRAM, &program);

  auto trans = glm::translate(glm::mat4(1.0f), t.translation);
  auto scale = glm::scale(glm::mat4(1.0f), t.scale);
  auto rot = glm::toMat4(glm::quat(glm::radians(t.rotation)));

  const auto model = trans * scale * rot;

  glUniformMatrix4fv(glGetUniformLocation(program, "model"), 1, GL_FALSE,
                     glm::value_ptr(model));
}

void render_points(const gl_object &g) {
  using gldm = gl_object::draw_mode;

  glBindVertexArray(g.vao);
  glPointSize(4.0f);

  if (g.dmode == gldm::points) {
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glDrawArrays(GL_POINTS, 0, g.points.size());
  } else if (g.dmode == gldm::lines) {
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glDrawElements(GL_LINES, g.indices.size(), GL_UNSIGNED_INT, NULL);
  } else if (g.dmode == gldm::triangles) {
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glDrawElements(GL_TRIANGLES, g.indices.size(), GL_UNSIGNED_INT, NULL);
  }
  glBindVertexArray(0);
}

glm::vec4 sample_torus(const torus_params &tp, const float u, const float v) {
  const auto sin_u = sinf(u);
  const auto cos_u = cosf(u);

  const auto sin_v = sinf(v);
  const auto cos_v = cosf(v);

  return {tp.radii[1] * cos_u + tp.radii[0] * cos_u * cos_v,
          tp.radii[1] * sin_u + tp.radii[0] * sin_u * cos_v,
          tp.radii[0] * sin_v, 1.0f};
}

void generate_lines(const parametric &p, const std::vector<glm::vec4> &points,
                    std::vector<unsigned int> &indices) {
  for (unsigned int i = 0u; i < p.samples[0]; ++i) {
    for (unsigned int j = 0u; j < p.samples[1]; ++j) {
      // add quad as two triangles
      const auto imod = (i + 1) % p.samples[0];
      const auto jmod = (j + 1) % p.samples[1];

      const auto i1 = i * p.samples[1] + j;
      const auto i2 = imod * p.samples[1] + j;
      const auto i3 = imod * p.samples[1] + jmod;
      const auto i4 = i * p.samples[1] + jmod;

      indices.push_back(i1);
      indices.push_back(i2);

      indices.push_back(i2);
      indices.push_back(i3);

      indices.push_back(i3);
      indices.push_back(i4);

      indices.push_back(i4);
      indices.push_back(i1);
    }
  }
}

} // namespace systems
