#version 460
layout( isolines ) in;

layout (std140) uniform common_block {
    mat4 proj;
    mat4 view;
    mat4 col_mat;
};

in vec4 tes_colors[];
out vec4 color;

void main() {
    vec3 p00 = vec3(gl_in[12].gl_Position);
    vec3 p10 = vec3(gl_in[13].gl_Position);
    vec3 p20 = vec3(gl_in[14].gl_Position);
    vec3 p30 = vec3(gl_in[15].gl_Position);
    vec3 p01 = vec3(gl_in[8].gl_Position);
    vec3 p11 = vec3(gl_in[9].gl_Position);
    vec3 p21 = vec3(gl_in[10].gl_Position);
    vec3 p31 = vec3(gl_in[11].gl_Position);
    vec3 p02 = vec3(gl_in[4].gl_Position);
    vec3 p12 = vec3(gl_in[5].gl_Position);
    vec3 p22 = vec3(gl_in[6].gl_Position);
    vec3 p32 = vec3(gl_in[7].gl_Position);
    vec3 p03 = vec3(gl_in[0].gl_Position);
    vec3 p13 = vec3(gl_in[1].gl_Position);
    vec3 p23 = vec3(gl_in[2].gl_Position);
    vec3 p33 = vec3(gl_in[3].gl_Position);

    float u = gl_TessCoord.x;
    float v = gl_TessCoord.y;

    float bu0 = (1.0 -u) * (1.0 -  u) * (1.0 - u);
    float bu1 = 3.0 * u * (1.0 -  u) * (1.0 - u);
    float bu2 = 3.0 * u * u * (1.0 - u);
    float bu3 = u * u * u;

    float bv0 = (1.0 -v) * (1.0 -  v) * (1.0 - v);
    float bv1 = 3.0 * v * (1.0 -  v) * (1.0 - v);
    float bv2 = 3.0 * v * v * (1.0 - v);
    float bv3 = v * v * v;

    gl_Position = proj * view * vec4(bu0 * (bv0*p00 + bv1 *p01 + bv2 *p02 + bv3 * p03)
        + bu1 * (bv0*p10 + bv1 *p11 + bv2 *p12 + bv3 * p13)
        + bu2 * (bv0*p20 + bv1 *p21 + bv2 *p22 + bv3 * p23)
        + bu3 * (bv0*p30 + bv1 *p31 + bv2 *p32 + bv3 * p33), 1.0);

    color = tes_colors[0];
}
