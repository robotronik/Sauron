#pragma once

#include <string>

#include <GL/glew.h>
#include <glm/glm.hpp>

struct Shader
{
	GLuint ProgramID;
	bool Loaded = false;

	bool LoadShader(std::string Vertex, std::string Fragment);
};
