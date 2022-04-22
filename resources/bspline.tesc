#version 460

layout (vertices = 4) out;

layout (std140) uniform common_block {
    mat4 proj;
    mat4 view;
};

in vec4 color[];
out vec4 tes_colors[];

void main() {
    vec4 o1 = (proj * view * gl_in[0].gl_Position);
    o1 = o1 / o1.w;
    vec4 o2 = (proj * view * gl_in[1].gl_Position);
    o2 = o2 / o2.w;
    vec4 o3 = (proj * view * gl_in[2].gl_Position);
    o3 = o3 / o3.w;
    vec4 o4 = (proj * view * gl_in[3].gl_Position);
    o4 = o4 / o4.w;

    float score = clamp(20 * (length(o2.xy - o1.xy) + length(o3.xy - o2.xy) + length(o4.xy - o1.xy)), 10, 100);

    gl_out[gl_InvocationID].gl_Position = gl_in[gl_InvocationID].gl_Position;
    tes_colors[gl_InvocationID] = color[gl_InvocationID];
    gl_TessLevelOuter[0] = 1.0;
    gl_TessLevelOuter[1] = score;
}
