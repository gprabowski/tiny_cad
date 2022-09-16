#version 460

layout(location = 0) in vec4 pos;
layout(location = 8) in vec4 col;

layout (std140) uniform common_block {
    mat4 proj;
    mat4 view;
    mat4 col_mat;
};

uniform mat4 model;

out vec4 color;

void main() {
    gl_Position = model * pos;
    color = col_mat * col;
}
