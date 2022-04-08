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
} // namespace systems
