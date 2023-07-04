#version 330 core

layout(location = 0) in vec4 position;
layout(location = 1) in vec3 normal;

uniform mat4 projMatrix;
uniform mat4 viewMatrix;
uniform mat4 modelMatrix;

out vec3 pass_normal;

void main()
{
    gl_Position = projMatrix * viewMatrix * modelMatrix * position;
    
    pass_normal = normal;
}
