#pragma once

#include <vector>

#include <glad/glad.h>

#include <gl_object.h>

struct fb_desc {
  uint32_t width{1000};
  uint32_t height{1000};
};

struct framebuffer {
  static framebuffer &get() {
    static framebuffer fb;
    return fb;
  }

  void bind();
  void setup_buffer(GLuint &color, GLuint &depth, GLuint &height);
  void setup_stereo_textures();

  void set_left();
  void set_right();
  void set_regular();

  void unbind();
  void revert_back();
  void invalidate();
  GLuint get_color_att();

  void switch_heights() {
    if (current_height.has_value()) {
      auto tmp = current_height.value();
      glDeleteTextures(1, &tmp);
    }
    current_height = of_fb_height_tex;
    glCreateTextures(GL_TEXTURE_2D, 1, &of_fb_height_tex);
  };

  GLuint get_height_att() { return of_fb_height_tex; };

  fb_desc &get_desc() { return desc; };

  gl_object quad;
  GLuint of_fb_col_tex_right;
  GLuint of_fb_col_tex_left;
  std::optional<GLuint> current_height;

private:
  fb_desc desc;
  GLuint of_fb;
  GLuint of_fb_col_tex;
  GLuint of_fb_dep_tex;
  GLuint of_fb_height_tex;

  std::vector<GLuint> additional_color;
  framebuffer();
  ~framebuffer();
};
