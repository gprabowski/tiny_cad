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

vec3 casteljau(float t, inout vec3 a, inout vec3 b, inout vec3 c, inout vec3 d) {
    vec3 e = (1.0 - t) * a + t * b;
    vec3 f = (1.0 - t) * b + t * c;
    vec3 g = (1.0 - t) * c + t * d;

    vec3 h = (1.0 - t) * e + t * f;
    vec3 i = (1.0 - t) * f + t * g;

    return (1.0 - t) * h + t * i;
}

void main() {
    vec3 p00 = vec3(gl_in[0].gl_Position);
    vec3 p01 = vec3(gl_in[1].gl_Position);
    vec3 p02 = vec3(gl_in[2].gl_Position);
    vec3 p03 = vec3(gl_in[3].gl_Position);
    vec3 p10 = vec3(gl_in[4].gl_Position);
    vec3 p11 = vec3(gl_in[5].gl_Position);
    vec3 p12 = vec3(gl_in[6].gl_Position);
    vec3 p13 = vec3(gl_in[7].gl_Position);
    vec3 p20 = vec3(gl_in[8].gl_Position);
    vec3 p21 = vec3(gl_in[9].gl_Position);
    vec3 p22 = vec3(gl_in[10].gl_Position);
    vec3 p23 = vec3(gl_in[11].gl_Position);
    vec3 p30 = vec3(gl_in[12].gl_Position);
    vec3 p31 = vec3(gl_in[13].gl_Position);
    vec3 p32 = vec3(gl_in[14].gl_Position);
    vec3 p33 = vec3(gl_in[15].gl_Position);

    float u = gl_TessCoord.x;
    float v = gl_TessCoord.y;

    vec3 inter_1 = casteljau(u, p00, p01, p02, p03);
    vec3 inter_2 = casteljau(u, p10, p11, p12, p13);
    vec3 inter_3 = casteljau(u, p20, p21, p22, p23);
    vec3 inter_4 = casteljau(u, p30, p31, p32, p33);

    vec3 result = casteljau(v, inter_1, inter_2, inter_3, inter_4);

    gl_Position = proj * view * vec4(result, 1.0);

    color = tes_colors[0];
    uv_coords = vec2(u, v);

    primitive_id = primitive_ids[0];
}
