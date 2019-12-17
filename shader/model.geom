#version 440 core
layout(triangles) in;
layout(triangle_strip, max_vertices=3) out;
layout(location = 0) in vs_gs{
    in vec4 position;
}gs_in[];

layout(location = 0) out vec3 normal;
layout(location = 1) out vec3 fragPos;

void main()
{
    vec3 pos[2];
    pos[0] = gs_in[1].position.xyz - gs_in[0].position.xyz;
    pos[1] = gs_in[2].position.xyz - gs_in[0].position.xyz;
    normal = cross(pos[0],pos[1]);
    for(int i=0; i< gl_in.length(); ++i)
    {
	normal = normalize(normal);
        gl_Position = gl_in[i].gl_Position;
        fragPos =  gs_in[i].position.xyz;
        EmitVertex();
    }
    EndPrimitive();
}
