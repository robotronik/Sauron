#pragma once

#include <string>

#include <vector>
#include <GL/glew.h>
#include <glm/glm.hpp>

#include "visualisation/openGL/Texture.hpp"

struct Mesh
{
private:
	GLuint PositionBuffer;
	GLuint UVBuffer;
	GLuint NormalBuffer;
	GLuint ColorBuffer;

	GLuint IndexBuffer;

public:
	std::vector<GLfloat> Positions;
	std::vector<GLfloat> UVs;
	std::vector<GLfloat> Normals;
	std::vector<GLfloat> Colors;

	std::vector<unsigned int> Indices;

	Texture texture;

	bool LoadFromFile(std::string path, std::string texturepath = "");

	void BindMesh();

	void Draw(GLuint ParamHandle = UINT32_MAX);
};
