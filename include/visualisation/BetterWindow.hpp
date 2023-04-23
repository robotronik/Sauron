#pragma once

#include <opencv2/core.hpp>
#include <glm/glm.hpp>
#include "visualisation/GLWindow.hpp"
#include "visualisation/openGL/Shader.hpp"

class BetterWindow : public GLWindow
{
private:
	GLuint VertexArrayID = 0;
	Shader ShaderProgram;
	GLuint ImageIDs = 0;
	GLuint NumImagesPrev = 0;
public:
	BetterWindow();
	~BetterWindow();

	void SetNumImages(int number);

	void SetImage(int number, cv::Mat image);
};


