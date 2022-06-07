#version 460

layout (vertices = 20) out;

layout (std140) uniform common_block {
    mat4 proj;
    mat4 view;
    mat4 col_mat;
};

uniform vec4 tess_outer;
uniform vec2 tess_inner;

in vec4 color[];
out vec4 tes_colors[];

void main() {
    gl_out[gl_InvocationID].gl_Position = gl_in[gl_InvocationID].gl_Position;

    tes_colors[gl_InvocationID] = color[gl_InvocationID];

    gl_TessLevelOuter[0] = tess_outer[0];
    gl_TessLevelOuter[2] = tess_outer[1];
    gl_TessLevelOuter[1] = tess_outer[2];
    gl_TessLevelOuter[3] = tess_outer[3];
    gl_TessLevelInner[0] = tess_inner[0];
    gl_TessLevelInner[1] = tess_inner[1];
}
