#version 460 core

in vec3 actualColor;
in vec2 TexCoord;

out vec4 FragColor;

void main() {

    FragColor = vec4(actualColor, 1.0);

}