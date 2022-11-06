#version 460

in vec4 color;
in vec2 torus_uv;
in vec3 world_pos;

layout (location = 0) out vec4 frag_color;
layout (location = 1) out float height;

uniform sampler2D trim_texture;
uniform vec4 trim_info;

void main() {
    if(trim_info.x > 0.5) {
      float texr = texture(trim_texture, torus_uv).r;
      if(((trim_info.y > 0.5f) && (texr < 0.5)) || ((trim_info.y <= 0.5) && (texr > 0.5))) {
        discard;
      }
    }
    frag_color = color;
}
