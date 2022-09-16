#version 460

in vec4 color;
in vec2 uv_coords;
flat in int primitive_id;

out vec4 frag_color;

uniform sampler2D trim_texture;
uniform vec4 trim_info;

uniform int patches_x;
uniform int patches_y;

void main() {
    if(trim_info.x > 0.5) {
      int x_idx = primitive_id % patches_x;
      int y_idx = primitive_id / patches_x;

      vec2 final_uv = vec2(uv_coords.x * 1.0/patches_x, uv_coords.y * 1.0/patches_y);
      final_uv = final_uv + vec2(x_idx * 1.0/patches_x, y_idx * 1.0/patches_y);

      if(texture(trim_texture, final_uv).r < 0.5) 
        discard;
    }
    frag_color = color;
}
