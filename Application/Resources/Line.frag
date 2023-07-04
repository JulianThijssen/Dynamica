#version 330 core

in vec3 pass_normal;

out vec4 fragColor;

void main()
{
    fragColor = vec4(pass_normal * 0.5 + 0.5, 1);
}
