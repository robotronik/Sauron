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
	std::vector<GLfloat> Normals;
	GLuint NormalBuffer;

	std::vector<unsigned int> Indices;
	GLuint IndexBuffer;

	bool LoadFromFile(std::string path);

	void BindMesh();

	void Draw();
};
