#version 460
layout( isolines ) in;

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
    float t = gl_TessCoord.x;

    vec3 b_a = gl_in[0].gl_Position.xyz;
    vec3 b_b = gl_in[1].gl_Position.xyz;
    vec3 b_c = gl_in[2].gl_Position.xyz;
    vec3 b_d = gl_in[3].gl_Position.xyz;

    vec3 result = casteljau(t, b_a, b_b, b_c, b_d);
    vec4 res = vec4(result, 1.0f);

    gl_Position = proj * view * res;

    color = tes_colors[0];
}
