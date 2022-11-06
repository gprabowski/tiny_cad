#version 460

in vec4 color;
layout (location = 0) out vec4 frag_color;
layout (location = 1) out vec4 height;

void main() {
    frag_color = color;
    height = vec4(0.f, 0.f, 0.f, 0.f);
}
