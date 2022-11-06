#version 460

in vec2 tex;

layout (location = 0) out vec4 frag_color;

uniform sampler2D leftTexture;
uniform sampler2D rightTexture;

void main() {
    frag_color = texture(leftTexture, tex) + texture(rightTexture, tex);
}
