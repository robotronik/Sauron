#include "visualisation/GLWindow.hpp"

#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

using namespace std;

bool GLWindow::OpenGLInitialized = false;
map<GLFWwindow*, GLWindow*> GLWindow::windowmap;

void window_size_callback_generic(GLFWwindow* window, int width, int height)
{
	GLWindow* target = GLWindow::windowmap.at(window);
	target->WindowSizeCallback(width, height);
}

void GLWindow::GLInit()
{
	if (OpenGLInitialized)
	{
		return;
	}
	//cout << "Creating OpenGL context" << endl;
	OpenGLInitialized = true;

	// Initialise GLFW
	glewExperimental = true; // Needed for core profile
	if( !glfwInit() )
	{
		cerr << "Failed to initialize GLFW" << endl;
		return;
	}
}

GLWindow::~GLWindow()
{
	if (Window != nullptr)
	{
		windowmap.erase(Window);
		glfwDestroyWindow(Window);
	}
	
}

GLFWwindow* GLWindow::GLCreateWindow(int width, int height, std::string name)
{
	if (Window != nullptr)
	{
		return Window;
	}
	GLInit();
	glfwWindowHint(GLFW_SAMPLES, 4); // 4x antialiasing
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3); // We want OpenGL 3.3
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // To make MacOS happy; should not be needed
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); // We don't want the old OpenGL 

	// Open a window and create its OpenGL context
	Window = glfwCreateWindow( width, height, name.c_str(), NULL, NULL);
	if( Window == NULL ){
		cerr << "Failed to open GLFW window. If you have an Intel GPU, they are not 3.3 compatible. Try the 2.1 version of the tutorials." << endl;
		glfwTerminate();
		return nullptr;
	}
	glfwMakeContextCurrent(Window); // Initialize GLEW
	if (glewInit() != GLEW_OK) {
		cerr << "Failed to initialize GLEW" << endl;
		return nullptr;
	}

	glfwSwapInterval( 0 ); //disable vsync

	// Enable depth test
	glEnable(GL_DEPTH_TEST);
	// Cull triangles which normal is not towards the camera
	glEnable(GL_CULL_FACE);
	// Accept fragment if it closer to the camera than the former one
	glDepthFunc(GL_LESS);

	glfwSetInputMode(Window, GLFW_STICKY_KEYS, GL_TRUE);
	glfwSetInputMode(Window, GLFW_RAW_MOUSE_MOTION, GL_TRUE);

	glfwSetWindowSizeCallback(Window, window_size_callback_generic);
	windowmap[Window] = this;
	return Window;
}

void GLWindow::WindowSizeCallback(int width, int height)
{

}