#pragma once
#include <glad/gl.h>

class Mesh
{
public:
	GLsizei vertexCount = 0;
	GLuint VAO = 0;
	GLuint VBO = 0;
private:
	virtual void SetupBuffers() = 0;
};