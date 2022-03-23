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

  enum class draw_mode { points, lines, triangles } dmode{draw_mode::lines};
  enum class vertex_t {
    point,
    point_color,
    point_normal,
    point_color_normal
  } vtype{vertex_t::point};
};
