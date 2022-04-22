#version 460
layout( isolines ) in;

layout (std140) uniform common_block {
    mat4 proj;
    mat4 view;
};

in vec4 tes_colors[];
out vec4 color;

void main() {
    vec3 P10 = gl_in[0].gl_Position.xyz;
    vec3 P20 = gl_in[1].gl_Position.xyz;
    vec3 P30 = gl_in[2].gl_Position.xyz;
    vec3 P40 = gl_in[3].gl_Position.xyz;

    float t = gl_TessCoord.x;

    const float u = 2.f + t;

    const float a41 = (u - 2.0f) / 3.0f;
    const float a31 = (u - 1.0f) / 3.0f;
    const float a21 = (u) / 3.0f;

    const float b41 = 1.0f - a41;
    const float b31 = 1.0f - a31;
    const float b21 = 1.0f - a21;

    const vec3 P41 = a41 * P40 + b41 * P30;
    const vec3 P31 = a31 * P30 + b31 * P20;
    const vec3 P21 = a21 * P20 + b21 * P10;

    const float a42 = (u - 2.0f) / 2.0f;
    const float a32 = (u - 1.0f) / 2.0f;

    const float b42 = 1.0f - a42;
    const float b32 = 1.0f - a32;

    const vec3 P42 = a42 * P41 + b42 * P31;
    const vec3 P32 = a32 * P31 + b32 * P21;

    const float a43 = (u - 2.0f) / 1.0f;
    const float b43 = 1.0f - a43;

    const vec3 P43 = a43 * P42 + b43 * P32;

    gl_Position = proj * view * vec4(P43, 1.0);

    color = tes_colors[0];
}
