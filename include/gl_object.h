#pragma once
#include <glad/glad.h>

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <vector>
#include <optional>

struct gl_object {
  GLuint program;
  GLuint vao{0}, vbo{0}, ebo{0};

  std::vector<glm::vec4> points;
  std::vector<unsigned int> indices;

  glm::vec4 color{0.9, 0.9, 0.9f, 1.0f};
  glm::vec4 primary{0.9, 0.9, 0.9f, 1.0f};
  glm::vec4 selected{1.0f, 0.4f, 0.4f, 1.0f};

  glm::vec4 tesselation_outer{4, 4, 4, 4};
  glm::vec2 tesselation_inner{4, 4};

  // 1. is it ON (1.0) or off (0.0)
  // 2. Is it regular (0.0)NO (1.0)YES
  glm::vec4 trimming_info {0.f, 1.f, 0.f, 0.f};
  std::optional<GLuint> trim_texture;

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
