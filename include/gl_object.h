#pragma once
#include <glad/glad.h>

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <vector>

struct gl_object {
  GLuint program;
  GLuint vao, vbo, ebo;

  std::vector<glm::vec4> points;
  std::vector<unsigned int> indices;

  glm::vec4 color{0.0f, 0.0f, 1.0f, 1.0f};
  glm::vec4 primary{0.0f, 0.0f, 1.0f, 1.0f};
  glm::vec4 selected{1.0f, 0.0f, 0.0f, 1.0f};

  enum class draw_mode {
    points,
    lines,
    line_strip,
    triangles
  } dmode{draw_mode::lines};

  enum class vertex_t {
    point,
    point_color,
    point_normal,
    point_color_normal
  } vtype{vertex_t::point};
};
