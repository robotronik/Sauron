#pragma once
#include <string>
#include <map>
#include <glm/glm.hpp>
class GLFWwindow;

class GLWindow
{
protected:
	static bool OpenGLInitialized;
	void GLInit();

public:
	static std::map<GLFWwindow*, GLWindow*> windowmap;
	GLFWwindow* Window = nullptr;

	virtual ~GLWindow();

	/*
	Creates a window.
	Context is set to be the newly created window
	*/
	GLFWwindow* GLCreateWindow(int width, int height, std::string name);

	virtual void WindowSizeCallback(int width, int height);
};