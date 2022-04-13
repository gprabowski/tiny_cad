#include <frame_state.h>
#include <framebuffer.h>
#include <log.h>

void framebuffer::invalidate() {
  if (!glIsFramebuffer(of_fb)) {
    glCreateFramebuffers(1, &of_fb);
  }

  if (glIsTexture(of_fb_col_tex)) {
    GLuint textures[] = {of_fb_col_tex, of_fb_dep_tex};
    glNamedFramebufferTexture(of_fb, GL_COLOR_ATTACHMENT0, 0, 0);
    glNamedFramebufferTexture(of_fb, GL_DEPTH_STENCIL_ATTACHMENT, 0, 0);
    glDeleteTextures(2, textures);
  }

  glCreateTextures(GL_TEXTURE_2D, 1, &of_fb_col_tex);
  glCreateTextures(GL_TEXTURE_2D, 1, &of_fb_dep_tex);

  // color
  glTextureParameteri(of_fb_col_tex, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTextureParameteri(of_fb_col_tex, GL_TEXTURE_WRAP_T, GL_REPEAT);
  glTextureParameteri(of_fb_col_tex, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTextureParameteri(of_fb_col_tex, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTextureStorage2D(of_fb_col_tex, 1, GL_RGBA8, desc.width, desc.height);
  // depth
  glTextureParameteri(of_fb_col_tex, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTextureParameteri(of_fb_col_tex, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTextureParameteri(of_fb_col_tex, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTextureParameteri(of_fb_col_tex, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTextureStorage2D(of_fb_dep_tex, 1, GL_DEPTH24_STENCIL8, desc.width,
                     desc.height);

  // final setup
  glNamedFramebufferTexture(of_fb, GL_COLOR_ATTACHMENT0, of_fb_col_tex, 0);
  glNamedFramebufferTexture(of_fb, GL_DEPTH_STENCIL_ATTACHMENT, of_fb_dep_tex,
                            0);
  if (glCheckNamedFramebufferStatus(of_fb, GL_FRAMEBUFFER) !=
      GL_FRAMEBUFFER_COMPLETE) {
    TINY_CAD_CRITICAL("Framebuffer creation failed!");
  } else {
    TINY_CAD_INFO("Successfull framebuffer creation!");
  }
  GLenum draw_bufs[] = {GL_COLOR_ATTACHMENT0};
  glNamedFramebufferDrawBuffers(of_fb, 1, draw_bufs);
};

framebuffer::framebuffer() {}

framebuffer::~framebuffer() { glDeleteFramebuffers(1, &of_fb); }

void framebuffer::bind() { glBindFramebuffer(GL_FRAMEBUFFER, of_fb); }
void framebuffer::unbind() { glBindFramebuffer(GL_FRAMEBUFFER, 0); }
GLuint framebuffer::get_color_att() { return of_fb_col_tex; }
