#version 330 core
in vec3 Normal;
in vec3 FragPos;
in vec3 Color;

out vec4 FragColor;

uniform vec3 lightPos;
uniform vec3 lightColor;
uniform vec3 viewPos;

void main()
{
    // Compute lighting (basic diffuse)
    vec3 norm = normalize(Normal);
    vec3 lightDir = normalize(lightPos - FragPos);
    float diff = max(dot(norm, lightDir), 0.0);

    vec3 lighting = lightColor * diff * Color;
    FragColor = vec4(lighting, 1.0);
}
