#version 440 core

layout(location = 0) in vec3 vertex;
layout(location = 0) out vs_gs{
    out vec4 position;
}vs_out;

layout(std140, binding = 0) uniform buf {
    mat4 mvp;
    mat4 mv;
} ubuf;

void main()
{
    gl_Position = ubuf.mvp * vec4(vertex,1.0);
    vs_out.position = ubuf.mv * vec4(vertex,1.0);
}
