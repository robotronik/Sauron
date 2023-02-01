#pragma once

#include <string>

#include <vector>
#include <GL/glew.h>
#include <glm/glm.hpp>

#include "visualisation/openGL/Texture.hpp"

//A 3D model
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

	bool LoadMesh(std::string path);

	bool LoadTexture(cv::Mat Texture);
	bool LoadTexture(std::string path);

	bool LoadFromFile(std::string path, std::string texturepath = "");

	void BindMesh(); //Create buffers for OpenGL

	void Draw(GLuint ParamHandle = UINT32_MAX, bool forceTexture = false); //Draw the mesh
};
