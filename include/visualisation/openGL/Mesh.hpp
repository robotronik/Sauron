#pragma once

#include <string>

#include <vector>
#include <GL/glew.h>
#include <glm/glm.hpp>

struct Mesh
{
	std::vector<GLfloat> Positions;
	GLuint PositionBuffer;
	std::vector<GLfloat> Colors;
	GLuint ColorBuffer;

	bool LoadFromFile(std::string path);

	void BindMesh();

	void Draw();
};
