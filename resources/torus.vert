#version 460

layout(location = 0) in vec4 pos;
layout(location = 1) in vec2 uv;
layout(location = 8) in vec4 col;

layout (std140) uniform common_block {
    mat4 proj;
    mat4 view;
    mat4 col_mat;
};

uniform mat4 model;

out vec4 color;
out vec2 torus_uv;
out vec3 world_pos;

void main() {
    gl_Position = proj * view * model * pos;
    color = col_mat * col;
    torus_uv = uv;
    world_pos = vec3(model * pos);
}
