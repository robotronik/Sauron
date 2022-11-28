#pragma once

#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include <GL/glew.h>
#include <glm/glm.hpp>

struct Texture
{
private:
	GLuint TextureID;

public:
	cv::Mat Texture;

	bool valid = false;

	void LoadFromFile(std::string path);

	void Bind();

	void Draw();
};
