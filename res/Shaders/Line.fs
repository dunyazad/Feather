#version 330 core
in vec3 FragPos;
in vec3 Color;

out vec4 FragColor;

uniform vec3 viewPos;

void main()
{
    FragColor = vec4(Color, 1.0);
}
