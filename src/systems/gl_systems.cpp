#include <systems.h>

namespace systems {
void render_gl(const gl_object &g) {
  using gldm = gl_object::draw_mode;

  glBindVertexArray(g.vao);
  glPointSize(4.0f);

  if (g.dmode == gldm::points) {
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glDrawArrays(GL_POINTS, 0, g.points.size());
  } else if (g.dmode == gldm::lines) {
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glDrawElements(GL_LINES, g.indices.size(), GL_UNSIGNED_INT, NULL);
  } else if (g.dmode == gldm::line_strip) {
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glDrawElements(GL_LINE_STRIP, g.indices.size(), GL_UNSIGNED_INT, NULL);
  } else if (g.dmode == gldm::patches) {
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glPatchParameteri(GL_PATCH_VERTICES, g.patch_size);
    glDrawElements(GL_PATCHES, g.indices.size(), GL_UNSIGNED_INT, NULL);
  } else if (g.dmode == gldm::triangles) {
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glDrawElements(GL_TRIANGLES, g.indices.size(), GL_UNSIGNED_INT, NULL);
  }
  glBindVertexArray(0);
  glPointSize(1.0f);
}
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

  switch (g.vtype) {
  case gl_object::vertex_t::point: {
    glVertexArrayVertexBuffer(g.vao, 0, g.vbo, 0, sizeof(g.points[0]));
    glVertexArrayElementBuffer(g.vao, g.ebo);
    glEnableVertexArrayAttrib(g.vao, 0);
    glVertexArrayAttribFormat(g.vao, 0, 4, GL_FLOAT, GL_FALSE, 0);
    glVertexArrayAttribBinding(g.vao, 0, 0);
  } break;
  case gl_object::vertex_t::point_tex: {
    glVertexArrayVertexBuffer(g.vao, 0, g.vbo, 0, 2 * sizeof(g.points[0]));
    glVertexArrayElementBuffer(g.vao, g.ebo);
    glEnableVertexArrayAttrib(g.vao, 0);
    glEnableVertexArrayAttrib(g.vao, 1);

    glVertexArrayAttribFormat(g.vao, 0, 4, GL_FLOAT, GL_FALSE, 0);
    glVertexArrayAttribFormat(g.vao, 1, 2, GL_FLOAT, GL_FALSE,
                              sizeof(glm::vec4));

    glVertexArrayAttribBinding(g.vao, 0, 0);
    glVertexArrayAttribBinding(g.vao, 1, 0);
  } break;
  case gl_object::vertex_t::point_color: {
    glVertexArrayVertexBuffer(g.vao, 0, g.vbo, 0, 2 * sizeof(g.points[0]));
    glVertexArrayElementBuffer(g.vao, g.ebo);
    glEnableVertexArrayAttrib(g.vao, 0);
    glEnableVertexArrayAttrib(g.vao, 1);

    glVertexArrayAttribFormat(g.vao, 0, 4, GL_FLOAT, GL_FALSE, 0);
    glVertexArrayAttribFormat(g.vao, 1, 4, GL_FLOAT, GL_FALSE,
                              sizeof(glm::vec4));

    glVertexArrayAttribBinding(g.vao, 0, 0);
    glVertexArrayAttribBinding(g.vao, 1, 0);
  } break;

  default:
    break;
  }
}
} // namespace systems
