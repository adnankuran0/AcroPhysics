#version 460 core
in vec3 FragPos;
in vec3 Normal;
in vec2 TexCoord;

out vec4 FragColor;

const vec3 lightDir = vec3(-0.2, -1.0, -0.3);
uniform vec3 viewPos;
uniform sampler2D tex;

void main()
{
    float ambientStrength = 0.2;
    float specularStrength = 0.5;
    
    vec3 ambient = ambientStrength * vec3(1.0f);
    
    vec3 norm = normalize(Normal);
    vec3 lightDirection = normalize(-lightDir); 
    float diff = max(dot(norm, lightDirection), 0.0);
    vec3 diffuse = diff * vec3(1.0f) * 2.0f;
    
    vec3 viewDir = normalize(viewPos - FragPos);
    vec3 reflectDir = reflect(-lightDirection, norm);  
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32);
    vec3 specular = specularStrength * spec * vec3(1.0f);  
    
    vec3 result = (ambient + diffuse + specular);
    FragColor = texture(tex, TexCoord) * vec4(result, 1.0);
}