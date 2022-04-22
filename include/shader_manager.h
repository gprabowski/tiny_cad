#pragma once
#include <filesystem>
#include <map>
#include <string>

#include <glad/glad.h>

enum class shader_t {
  POINT_SHADER,
  CURSOR_SHADER,
  TORUS_SHADER,
  BEZIER_CURVE_SHADER,
  BSPLINE_CURVE_SHADER,
  INTERPOLATION_CURVE_SHADER,
};

struct shader {
  GLuint idx{0u};
  GLuint ubo_idx{0u};
};

struct shader_manager {
  static shader_manager &get_manager();
  ~shader_manager();

  shader_manager &operator=(const shader_manager &r) = delete;
  shader_manager &operator=(const shader_manager &&r) = delete;
  shader_manager(const shader_manager &) = delete;
  shader_manager(const shader_manager &&) = delete;

  GLuint add(shader_t st, const std::string &name);

  GLuint common_ubo{0};
  const int common_ubo_block_loc{0};

  std::map<shader_t, shader> programs;

private:
  shader_manager() = default;

  std::string
  read_shader_file(const std::filesystem::path::value_type *shader_file);

  std::map<std::string, shader> cache;
};
