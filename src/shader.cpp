#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <shader.h>

#include <glad/glad.h>

std::string
shader::read_shader_file(const std::filesystem::path::value_type *shader_file) {
  std::ifstream ifs;

  auto ex = ifs.exceptions();
  ex |= std::ios_base::badbit | std::ios_base::failbit;
  ifs.exceptions(ex);

  ifs.open(shader_file);
  ifs.ignore(std::numeric_limits<std::streamsize>::max());
  auto size = ifs.gcount();

  ifs.clear();
  ifs.seekg(0, std::ios_base::beg);

  return std::string{std::istreambuf_iterator<char>{ifs}, {}};
}

GLuint shader::LoadProgram(const std::string &name) {
  std::string vert_source = read_shader_file((name + ".vert").c_str());
  std::string frag_source = read_shader_file((name + ".frag").c_str());

  const char *vert_src_ptr = vert_source.c_str();
  const char *frag_src_ptr = frag_source.c_str();

  GLuint vertex_shader = glCreateShader(GL_VERTEX_SHADER);
  glShaderSource(vertex_shader, 1, &vert_src_ptr, NULL);
  glCompileShader(vertex_shader);

  GLint vcompiled;
  glGetShaderiv(vertex_shader, GL_COMPILE_STATUS, &vcompiled);
  if (vcompiled != GL_TRUE) {
    GLsizei log_length = 0;
    GLchar message[1024];
    glGetShaderInfoLog(vertex_shader, 1024, &log_length, message);
    fprintf(stderr, "[VERT SHADER] %s\n", message);
  }

  GLuint frag_shader = glCreateShader(GL_FRAGMENT_SHADER);
  glShaderSource(frag_shader, 1, &frag_src_ptr, NULL);
  glCompileShader(frag_shader);

  GLint fcompiled;
  glGetShaderiv(frag_shader, GL_COMPILE_STATUS, &fcompiled);
  if (fcompiled != GL_TRUE) {
    GLsizei log_length = 0;
    GLchar message[1024];
    glGetShaderInfoLog(frag_shader, 1024, &log_length, message);
    fprintf(stderr, "[FRAG SHADER] %s\n", message);
  }

  GLuint program = glCreateProgram();

  glAttachShader(program, vertex_shader);
  glAttachShader(program, frag_shader);
  glLinkProgram(program);

  GLint plinked;
  glGetProgramiv(program, GL_LINK_STATUS, &plinked);
  if (plinked != GL_TRUE) {
    GLsizei log_length = 0;
    GLchar message[1024];
    glGetProgramInfoLog(program, 1024, &log_length, message);
    fprintf(stderr, "[PROG LINK] %s\n", message);
  }

  glDeleteShader(vertex_shader);
  glDeleteShader(frag_shader);

  return program;
}
