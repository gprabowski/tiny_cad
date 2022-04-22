#version 460
layout( isolines ) in;

layout (std140) uniform common_block {
    mat4 proj;
    mat4 view;
};

in vec4 tes_colors[];
out vec4 color;

void main() {
    float t = gl_TessCoord.x;

    vec3 b_a = gl_in[0].gl_Position.xyz;
    vec3 b_b = gl_in[1].gl_Position.xyz;
    vec3 b_c = gl_in[2].gl_Position.xyz;
    vec3 b_d = gl_in[3].gl_Position.xyz;

    vec3 b_e = (1.0 - t) * b_a + t * b_b;
    vec3 b_f = (1.0 - t) * b_b + t * b_c;
    vec3 b_g = (1.0 - t) * b_c + t * b_d;
    vec3 b_h = (1.0 - t) * b_e + t * b_f;
    vec3 b_i = (1.0 - t) * b_f + t * b_g;

    vec4 res = vec4((1.0 - t) * b_h + t * b_i, 1.0f);

    gl_Position = proj * view * res;

    color = tes_colors[0];
}
