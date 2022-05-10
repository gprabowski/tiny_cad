#version 460

layout(location = 0) in vec4 pos;
layout(location = 1) in vec2 texcoord;

layout (std140) uniform common_block {
    mat4 proj;
    mat4 view;
    mat4 col_mat;
};

uniform mat4 model;

out vec2 tex;

void main() {
    gl_Position = pos;
    tex = texcoord;
}
