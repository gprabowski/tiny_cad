#version 460
layout( quads, equal_spacing, ccw ) in;

layout (std140) uniform common_block {
    mat4 proj;
    mat4 view;
    mat4 col_mat;
};

in vec4 tes_colors[];
in int primitive_ids[];
out vec4 color;
out vec2 uv_coords;
flat out int primitive_id;

vec3 deboor(float t, inout vec3 P10, inout vec3 P20, inout vec3 P30, inout vec3 P40) {
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

    primitive_id = primitive_ids[0];

    return P43;
}

void main() {
    vec3 p00 = vec3(gl_in[0].gl_Position);
    vec3 p01 = vec3(gl_in[4].gl_Position);
    vec3 p02 = vec3(gl_in[8].gl_Position);
    vec3 p03 = vec3(gl_in[12].gl_Position);

    vec3 p10 = vec3(gl_in[1].gl_Position);
    vec3 p11 = vec3(gl_in[5].gl_Position);
    vec3 p12 = vec3(gl_in[9].gl_Position);
    vec3 p13 = vec3(gl_in[13].gl_Position);

    vec3 p20 = vec3(gl_in[2].gl_Position);
    vec3 p21 = vec3(gl_in[6].gl_Position);
    vec3 p22 = vec3(gl_in[10].gl_Position);
    vec3 p23 = vec3(gl_in[14].gl_Position);

    vec3 p30 = vec3(gl_in[3].gl_Position);
    vec3 p31 = vec3(gl_in[7].gl_Position);
    vec3 p32 = vec3(gl_in[11].gl_Position);
    vec3 p33 = vec3(gl_in[15].gl_Position);

    float u = gl_TessCoord.x;
    float v = gl_TessCoord.y;

    vec3 inter_1 = deboor(v, p00, p01, p02, p03);
    vec3 inter_2 = deboor(v, p10, p11, p12, p13);
    vec3 inter_3 = deboor(v, p20, p21, p22, p23);
    vec3 inter_4 = deboor(v, p30, p31, p32, p33);

    vec3 result = deboor(u, inter_1, inter_2, inter_3, inter_4);

    gl_Position = proj * view * vec4(result, 1.0);

    color = tes_colors[0];

    uv_coords = vec2(u, v);
}
