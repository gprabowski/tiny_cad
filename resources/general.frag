#version 460

in vec4 color;
out vec4 frag_color;

uniform sampler2D trim_texture;
uniform vec4 trim_info;

void main() {
    frag_color = color;
}
