#version 460 core
in vec3 FragPos;
in vec3 Normal;
in vec2 TexCoord;
in vec4 FragPosLightSpace;

out vec4 FragColor;

const vec3 lightDir = vec3(-0.2, -1.0, -0.3);
uniform vec3 viewPos;
uniform sampler2D tex;
uniform sampler2D shadowMap;

float ShadowFactor(vec4 fragPosLightSpace)
{
    vec3 proj = fragPosLightSpace.xyz / fragPosLightSpace.w;
    proj = proj * 0.5 + 0.5;

    if (proj.z > 1.0)
        return 0.0;

    float bias = max(0.005 * (1.0 - dot(normalize(Normal), normalize(-lightDir))), 0.001);

    float shadow = 0.0;
    vec2 texelSize = 1.0 / textureSize(shadowMap, 0);

    for (int x = -1; x <= 1; ++x)
    {
        for (int y = -1; y <= 1; ++y)
        {
            float pcfDepth = texture(
                shadowMap,
                proj.xy + vec2(x, y) * texelSize
            ).r;

            shadow += proj.z - bias > pcfDepth ? 1.0 : 0.0;
        }
    }

    shadow /= 9.0;
    return shadow;
}

void main()
{
    float ambientStrength = 0.3;
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
    float shadow = ShadowFactor(FragPosLightSpace);

    diffuse*= (1.0 - shadow);
    specular *= (1.0 - shadow);
    vec3 result = (ambient + diffuse + specular);
    FragColor = texture(tex, TexCoord) * vec4(result, 1.0);
}