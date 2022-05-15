#version 460

layout (vertices = 16) out;

layout (std140) uniform common_block {
    mat4 proj;
    mat4 view;
    mat4 col_mat;
};

in vec4 color[];
out vec4 tes_colors[];

void main() {
    gl_out[gl_InvocationID].gl_Position = gl_in[gl_InvocationID].gl_Position;

    tes_colors[gl_InvocationID] = color[gl_InvocationID];

    gl_TessLevelOuter[0] = gl_TessLevelOuter[2] = 10;
    gl_TessLevelOuter[1] = gl_TessLevelOuter[3] = 10;
    gl_TessLevelInner[0] = 10;
    gl_TessLevelInner[1] = 10;
}
