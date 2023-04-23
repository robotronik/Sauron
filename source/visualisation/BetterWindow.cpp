#include "visualisation/BetterWindow.hpp"
#include <string>

using namespace std;

static string shaderfolder = "../source/visualisation/openGL/";
BetterWindow::BetterWindow()
{
	GLCreateWindow(1280, 720, "Direct");
	// Create and compile our GLSL program from the shaders
	ShaderProgram.LoadShader(shaderfolder + "vertexshader.vs", shaderfolder + "fragmentshader.fs");

	glGenVertexArrays(1, &VertexArrayID);
	glBindVertexArray(VertexArrayID);
}

BetterWindow::~BetterWindow()
{
}

void BetterWindow::SetNumImages(int number)
{
	if (number == NumImagesPrev)
	{
		return;
	}
	if (NumImagesPrev != 0)
	{
		glDeleteTextures(NumImagesPrev, &ImageIDs);
	}
	glGenTextures(number, &ImageIDs);
	NumImagesPrev = number;
}

void BetterWindow::SetImage(int number, cv::Mat image)
{
	if (number > NumImagesPrev)
	{
		return;
	}
	
}