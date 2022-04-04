#version 460

layout(location = 0) in vec4 pos;
layout(location = 1) in vec4 col;

layout (std140) uniform common_block {
    mat4 proj;
    mat4 view;
};

uniform mat4 model;


out vec4 color;

void main() {
    gl_Position = proj * view * model * pos;
    color = col;
}
