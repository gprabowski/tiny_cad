#version 460

layout(location = 0) in vec4 pos;

layout (std140) uniform common_block {
    mat4 proj;
    mat4 view;
    mat4 col_mat;
};

float gridSize = 600.0f;
float gridCellSize = 0.05f;

vec4 gridColorThin = vec4(0.5, 0.5, 0.5, 1.0);
vec4 gridColorThick = vec4(0.0, 0.0, 0.0, 1.0);

const float gridMinPixelsBetweenCells = 2.0;

out vec2 uv;

void main() {
    vec3 vpos = vec3(pos) * gridSize;
    gl_Position = proj * view * vec4(vpos, 1.0);
    uv = vpos.xz;
}
