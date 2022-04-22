#include <fstream>
#include <log.h>
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
  // auto size = ifs.gcount();

  ifs.clear();
  ifs.seekg(0, std::ios_base::beg);

  return std::string{std::istreambuf_iterator<char>{ifs}, {}};
}

void bind_common_ubo(shader &s) {
  auto &sm = shader_manager::get_manager();
  s.ubo_idx = glGetUniformBlockIndex(s.idx, "common_block");
  glUniformBlockBinding(s.idx, s.ubo_idx, sm.common_ubo_block_loc);
}

GLuint shader_manager::add(shader_t st, const std::string &name) {
  GLuint ret;
  if (cache.count(name)) {
    programs[st] = cache.at(name);
    ret = programs[st].idx;
  } else {
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
      TINY_CAD_ERROR("[VERT SHADER] {0}", message);
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
      TINY_CAD_ERROR("[FRAG SHADER] {0}", message);
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
      TINY_CAD_ERROR("[PROG LINK] {0}", message);
    }

    glDeleteShader(vertex_shader);
    glDeleteShader(frag_shader);

    shader ret_s{program, 0u};
    bind_common_ubo(ret_s);

    cache[name] = ret_s;
    programs[st] = ret_s;

    ret = ret_s.idx;
  }

  return ret;
}
