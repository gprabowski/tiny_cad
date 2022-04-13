#pragma once

#include <glad/glad.h>

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
  void unbind();
  void revert_back();
  void invalidate();
  GLuint get_color_att();
  fb_desc &get_desc() { return desc; };

private:
  fb_desc desc;
  GLuint of_fb;
  GLuint of_fb_col_tex;
  GLuint of_fb_dep_tex;
  framebuffer();
  ~framebuffer();
};
