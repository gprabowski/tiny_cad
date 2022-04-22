#include <fstream>
#include <log.h>
#include <optional>
#include <shader_manager.h>

shader_manager &shader_manager::get_manager() {
  static shader_manager sm;
  return sm;
}

shader_manager::~shader_manager() {
  for (auto &[str, s] : cache) {
    glDeleteProgram(s.idx);
  }
}

std::string shader_manager::read_shader_file(
    const std::filesystem::path::value_type *shader_file) {
  std::ifstream ifs;

  auto ex = ifs.exceptions();
  ex |= std::ios_base::badbit | std::ios_base::failbit;
  ifs.exceptions(ex);

  ifs.open(shader_file);
  ifs.ignore(std::numeric_limits<std::streamsize>::max());
  auto size = ifs.gcount();
  TINY_CAD_INFO("[SHADER] Read {0} bytes from {1}", size, shader_file);

  ifs.clear();
  ifs.seekg(0, std::ios_base::beg);

  return std::string{std::istreambuf_iterator<char>{ifs}, {}};
}

void bind_common_ubo(shader &s) {
  auto &sm = shader_manager::get_manager();
  s.ubo_idx = glGetUniformBlockIndex(s.idx, "common_block");
  glUniformBlockBinding(s.idx, s.ubo_idx, sm.common_ubo_block_loc);
}

GLuint compile_shader_from_source(const std::string &source, GLuint type) {
  GLuint shader = glCreateShader(type);
  const char *src = source.c_str();
  glShaderSource(shader, 1, &src, NULL);
  glCompileShader(shader);

  GLint compiled;
  glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
  if (compiled != GL_TRUE) {
    GLsizei log_length = 0;
    GLchar message[1024];
    glGetShaderInfoLog(shader, 1024, &log_length, message);
    TINY_CAD_ERROR("[SHADER {0}] {1}", type, message);
  }

  return shader;
}

namespace fs = std::filesystem;

GLuint shader_manager::add(shader_t st, const std::string &name) {
  GLuint ret;
  if (cache.count(name)) {
    programs[st] = cache.at(name);
    ret = programs[st].idx;
  } else {
    // optional stages
    std::optional<GLuint> tcs_shader;
    std::optional<GLuint> tes_shader;

    std::string vert_source = read_shader_file((name + ".vert").c_str());
    std::string frag_source = read_shader_file((name + ".frag").c_str());

    if (fs::exists(name + ".tcs")) {
      std::string tcs_source = read_shader_file((name + ".tcs").c_str());
      tcs_shader =
          compile_shader_from_source(tcs_source, GL_TESS_CONTROL_SHADER);
    }

    if (fs::exists(name + ".tes")) {
      std::string tes_source = read_shader_file((name + ".tes").c_str());
      tes_shader =
          compile_shader_from_source(tes_source, GL_TESS_EVALUATION_SHADER);
    }

    GLuint vertex_shader =
        compile_shader_from_source(vert_source, GL_VERTEX_SHADER);
    GLuint frag_shader =
        compile_shader_from_source(frag_source, GL_FRAGMENT_SHADER);

    GLuint program = glCreateProgram();

    glAttachShader(program, vertex_shader);
    glAttachShader(program, frag_shader);

    if (tcs_shader.has_value()) {
      glAttachShader(program, tcs_shader.value());
    }
    if (tes_shader.has_value()) {
      glAttachShader(program, tes_shader.value());
    }

    glLinkProgram(program);

    GLint plinked;
    glGetProgramiv(program, GL_LINK_STATUS, &plinked);
    if (plinked != GL_TRUE) {
      GLsizei log_length = 0;
      GLchar message[1024];
      glGetProgramInfoLog(program, 1024, &log_length, message);
      TINY_CAD_ERROR("[PROG LINK] {0}", message);
    }

    glDeleteShader(vertex_shader);
    glDeleteShader(frag_shader);

    if (tcs_shader.has_value()) {
      glDeleteShader(tcs_shader.value());
    }

    if (tes_shader.has_value()) {
      glDeleteShader(tes_shader.value());
    }

    shader ret_s{program, 0u};
    bind_common_ubo(ret_s);

    cache[name] = ret_s;
    programs[st] = ret_s;

    ret = ret_s.idx;
  }

  return ret;
}
