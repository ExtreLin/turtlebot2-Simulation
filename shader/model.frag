#version 440 core

layout(location = 0) in vec3 normal;
layout(location = 1) in vec3 fragPos;

layout(std140, binding = 1) uniform buf{
     vec3 constcolor;
     vec3 eyePos;
}ubuf;

layout(location = 0 ) out vec4 outColor;


void main()
{
    vec3 tmpnormal = normal;

    vec3 lightColor = vec3(1.0, 1.0,1.0);
    vec3 lightPositon = vec3(0.0, 0.0,1000.0);
     float ambientStrength = 0.1;
    vec3 ambient = ambientStrength * lightColor;
	
    // diffuse 
    vec3 lightDir = normalize(ubuf.eyePos - fragPos);
    float diff = abs(dot(tmpnormal, lightDir));

    vec3 diffuse = diff * lightColor;
    
    // specular
    float specularStrength = 0.5;
    vec3 viewDir = normalize(ubuf.eyePos - fragPos);
    vec3 reflectDir = reflect(-lightDir, tmpnormal);  
    float spec = pow(abs(dot(viewDir, reflectDir)), 32);
    vec3 specular = specularStrength * spec * lightColor;  
        
    vec3 result = (ambient + diffuse + specular) * ubuf.constcolor;
    outColor = vec4(result, 1.0);
}

