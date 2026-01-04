#pragma once
#include <string>

class Texture
{
public:
	Texture() = default;
	Texture(const char* texturePath);
	void Init(const char* texturePath);
	void Bind(unsigned int slot);
private:
	unsigned int m_Texture{};
	int width{}, height{}, nrChannels{};

};