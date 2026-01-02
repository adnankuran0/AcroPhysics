#include "Texture.h"
#include <stb_image.h>
#include <glad/gl.h>

Texture::Texture(const std::string& texturePath)
{
    unsigned char* data = stbi_load(texturePath.c_str(), &width, &height, &nrChannels, 0);

    glGenTextures(1, &m_Texture);
    glBindTexture(GL_TEXTURE_2D, m_Texture);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

    GLenum format = GL_RGB;
    if (nrChannels == 4) format = GL_RGBA;
    else if (nrChannels == 1) format = GL_RED;

    glTexImage2D(
        GL_TEXTURE_2D,
        0,
        format,
        width,
        height,
        0,
        format,
        GL_UNSIGNED_BYTE,
        data
    );

    glGenerateMipmap(GL_TEXTURE_2D);
    stbi_image_free(data);
}

void Texture::Bind(unsigned int slot)
{
	glActiveTexture(GL_TEXTURE0 + slot);
}
