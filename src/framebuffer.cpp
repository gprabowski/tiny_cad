#include <frame_state.h>
#include <framebuffer.h>
#include <input_state.h>
#include <log.h>

#include <algorithm>

void framebuffer::setup_stereo_textures() {
  if (!glIsFramebuffer(of_fb)) {
    glCreateFramebuffers(1, &of_fb);
  }
  desc.width = std::max<float>(1.0f, desc.width);
  desc.height = std::max<float>(1.0f, desc.height);

  if (glIsTexture(of_fb_col_tex_left)) {
    GLuint textures[] = {of_fb_col_tex_left, of_fb_col_tex_right};
    glDeleteTextures(2, textures);
  }

  glCreateTextures(GL_TEXTURE_2D, 1, &of_fb_col_tex_left);
  glCreateTextures(GL_TEXTURE_2D, 1, &of_fb_col_tex_right);

  // color
  glTextureParameteri(of_fb_col_tex_left, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTextureParameteri(of_fb_col_tex_left, GL_TEXTURE_WRAP_T, GL_REPEAT);
  glTextureParameteri(of_fb_col_tex_left, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTextureParameteri(of_fb_col_tex_left, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTextureStorage2D(of_fb_col_tex_left, 1, GL_RGBA8, desc.width, desc.height);

  glTextureParameteri(of_fb_col_tex_right, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTextureParameteri(of_fb_col_tex_right, GL_TEXTURE_WRAP_T, GL_REPEAT);
  glTextureParameteri(of_fb_col_tex_right, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTextureParameteri(of_fb_col_tex_right, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTextureStorage2D(of_fb_col_tex_right, 1, GL_RGBA8, desc.width, desc.height);
}

void framebuffer::setup_buffer(GLuint color, GLuint depth) {
  if (!glIsFramebuffer(of_fb)) {
    glCreateFramebuffers(1, &of_fb);
  }
  desc.width = std::max<float>(1.0f, desc.width);
  desc.height = std::max<float>(1.0f, desc.height);

  if (glIsTexture(color)) {
    GLuint textures[] = {color, depth};
    glNamedFramebufferTexture(of_fb, GL_COLOR_ATTACHMENT0, 0, 0);
    glNamedFramebufferTexture(of_fb, GL_DEPTH_STENCIL_ATTACHMENT, 0, 0);
    glDeleteTextures(2, textures);
  }

  glCreateTextures(GL_TEXTURE_2D, 1, &color);
  glCreateTextures(GL_TEXTURE_2D, 1, &depth);

  // color
  glTextureParameteri(color, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTextureParameteri(color, GL_TEXTURE_WRAP_T, GL_REPEAT);
  glTextureParameteri(color, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTextureParameteri(color, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTextureStorage2D(color, 1, GL_RGBA8, desc.width, desc.height);

  // depth
  glTextureParameteri(depth, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTextureParameteri(depth, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTextureParameteri(depth, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTextureParameteri(depth, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTextureStorage2D(depth, 1, GL_DEPTH24_STENCIL8, desc.width, desc.height);

  // final setup
  glNamedFramebufferTexture(of_fb, GL_COLOR_ATTACHMENT0, color, 0);
  glNamedFramebufferTexture(of_fb, GL_DEPTH_STENCIL_ATTACHMENT, depth, 0);
  if (glCheckNamedFramebufferStatus(of_fb, GL_FRAMEBUFFER) !=
      GL_FRAMEBUFFER_COMPLETE) {
    TINY_CAD_CRITICAL("Framebuffer creation failed!");
  }
  GLenum draw_bufs[] = {GL_COLOR_ATTACHMENT0};
  glNamedFramebufferDrawBuffers(of_fb, 1, draw_bufs);
}

void framebuffer::set_left() {
  glNamedFramebufferTexture(of_fb, GL_COLOR_ATTACHMENT0, of_fb_col_tex_left, 0);
}
void framebuffer::set_right() {
  glNamedFramebufferTexture(of_fb, GL_COLOR_ATTACHMENT0, of_fb_col_tex_right,
                            0);
}
void framebuffer::set_regular() {
  glNamedFramebufferTexture(of_fb, GL_COLOR_ATTACHMENT0, of_fb_col_tex, 0);
}

void framebuffer::invalidate() {
  auto &is = input_state::get_input_state();
  setup_buffer(of_fb_col_tex, of_fb_dep_tex);
  if (is.stereo) {
    setup_stereo_textures();
  }
};

framebuffer::framebuffer() {}

framebuffer::~framebuffer() { glDeleteFramebuffers(1, &of_fb); }

void framebuffer::bind() { glBindFramebuffer(GL_FRAMEBUFFER, of_fb); }
void framebuffer::unbind() { glBindFramebuffer(GL_FRAMEBUFFER, 0); }
GLuint framebuffer::get_color_att() { return of_fb_col_tex; }
