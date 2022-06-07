#version 460
layout( quads, equal_spacing, ccw ) in;

layout (std140) uniform common_block {
    mat4 proj;
    mat4 view;
    mat4 col_mat;
};

in vec4 tes_colors[];
out vec4 color;

vec3 casteljau(float t, inout vec3 a, inout vec3 b, inout vec3 c, inout vec3 d) {
    vec3 e = (1.0 - t) * a + t * b;
    vec3 f = (1.0 - t) * b + t * c;
    vec3 g = (1.0 - t) * c + t * d;

    vec3 h = (1.0 - t) * e + t * f;
    vec3 i = (1.0 - t) * f + t * g;

    return (1.0 - t) * h + t * i;
}

void main() {
    vec3 p3 = vec3(gl_in[0].gl_Position);
    vec3 e3minus = vec3(gl_in[1].gl_Position);
    vec3 e2plus = vec3(gl_in[2].gl_Position);
    vec3 p2 = vec3(gl_in[3].gl_Position);

    vec3 e3plus = vec3(gl_in[4].gl_Position);
    vec3 f3_minus = vec3(gl_in[5].gl_Position);
    vec3 f3_plus = vec3(gl_in[6].gl_Position);
    vec3 f2_plus = vec3(gl_in[7].gl_Position);
    vec3 f2_minus = vec3(gl_in[8].gl_Position);
    vec3 e2 = vec3(gl_in[9].gl_Position);

    vec3 e0_minus = vec3(gl_in[10].gl_Position);
    vec3 f0_minus = vec3(gl_in[11].gl_Position);
    vec3 f0_plus = vec3(gl_in[12].gl_Position);
    vec3 f1_plus = vec3(gl_in[13].gl_Position);
    vec3 f1_minus = vec3(gl_in[14].gl_Position);
    vec3 e1_plus = vec3(gl_in[15].gl_Position);

    vec3 p0 = vec3(gl_in[16].gl_Position);
    vec3 e0_plus = vec3(gl_in[17].gl_Position);
    vec3 e1_minus = vec3(gl_in[18].gl_Position);
    vec3 p1 = vec3(gl_in[19].gl_Position);

    float u = gl_TessCoord.x;
    float v = gl_TessCoord.y;

    vec3 f0 = (u * f0_plus + v * f0_minus) / max((u + v), 0.1);
    vec3 f1 = ((1 - u) * f1_minus + v * f1_plus) / max((1 - u + v), 0.1);
    vec3 f2 = ((1 - u) * f2_plus + (1 - v) * f2_minus) / max((2 - u - v), 0.1);
    vec3 f3 = (u * f3_minus + (1 - v) * f3_plus) / max((1 + u - v), 0.1);

    vec3 inter_1 = casteljau(u, p3, e3minus, e2plus, p2);
    vec3 inter_2 = casteljau(u, e3plus, f3, f2, e2);
    vec3 inter_3 = casteljau(u, e0_minus, f0, f1, e1_plus);
    vec3 inter_4 = casteljau(u, p0, e0_plus, e1_minus, p1);

    vec3 result = casteljau(v, inter_1, inter_2, inter_3, inter_4);

    gl_Position = proj * view * vec4(result, 1.0);

    color = tes_colors[0];
}
