#pragma once
#include <glad/glad.h>

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <vector>

struct gl_object {
  GLuint program;
  GLuint vao{0}, vbo{0}, ebo{0};

  std::vector<glm::vec4> points;
  std::vector<unsigned int> indices;

  glm::vec4 color{0.0f, 0.0f, 1.0f, 1.0f};
  glm::vec4 primary{0.0f, 0.0f, 1.0f, 1.0f};
  glm::vec4 selected{1.0f, 0.0f, 0.0f, 1.0f};

  glm::vec4 tesselation_outer{3, 3, 3, 3};
  glm::vec2 tesselation_inner{3, 3};

  enum class draw_mode : int {
    points = 0,
    lines = 1,
    line_strip = 2,
    triangles = 3,
    patches = 4
  } dmode{draw_mode::lines};
  GLuint patch_size{1};

  enum class vertex_t {
    point,
    point_color,
    point_tex,
    point_normal,
    point_color_normal
  } vtype{vertex_t::point};

  ~gl_object() {
    if (glIsBuffer(vbo)) {
      glDeleteBuffers(1, &vbo);
    }
    if (glIsBuffer(ebo)) {
      glDeleteBuffers(1, &ebo);
    }
    if (glIsVertexArray(vao)) {
      glDeleteVertexArrays(1, &vao);
    }
  }
};
