#version 330 core
layout (location = 0) in vec3 aPos;      // Cube Vertex Position
layout (location = 1) in vec3 aNormal;   // Cube Normal
layout (location = 2) in vec3 aColor;    // Cube Color
layout (location = 3) in mat4 instanceMatrix; // Per-instance transformation

out vec3 Normal;
out vec3 FragPos;
out vec3 Color;

uniform mat4 view;
uniform mat4 projection;

void main()
{
    vec4 worldPos = instanceMatrix * vec4(aPos, 1.0);
    FragPos = worldPos.xyz;
    Normal = mat3(transpose(inverse(instanceMatrix))) * aNormal; // Transform normal
    Color = aColor;
    
    gl_Position = projection * view * worldPos;
}
