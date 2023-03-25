#include "visualisation/BoardGL.hpp"

#include <iostream>
#include <fstream>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>	

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/transform.hpp>
#include <assimp/Importer.hpp>

#include "visualisation/openGL/Mesh.hpp"
#include "TrackedObjects/TrackedObject.hpp"
#include "GlobalConf.hpp"

using namespace std;

bool BoardGL::HasInit = false;
GLuint BoardGL::VertexArrayID;
Shader BoardGL::ShaderProgram;
bool BoardGL::MeshesLoaded = false, BoardGL::TagsLoaded = false;
Mesh BoardGL::robot, BoardGL::arena, BoardGL::axis, BoardGL::brio, BoardGL::puck, BoardGL::tag, BoardGL::skybox;
std::vector<Texture> BoardGL::TagTextures;

string shaderfolder = "../source/visualisation/openGL/";

void BoardGL::GLInit()
{
	// Initialise GLFW
	glewExperimental = true; // Needed for core profile
	if( !glfwInit() )
	{
		cerr << "Failed to initialize GLFW" << endl;
		return;
	}
}

void window_size_callback(GLFWwindow* window, int width, int height)
{
	glViewport(0, 0, width, height);
}

GLFWwindow* BoardGL::GLCreateWindow(cv::Size windowsize)
{
	glfwWindowHint(GLFW_SAMPLES, 4); // 4x antialiasing
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3); // We want OpenGL 3.3
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // To make MacOS happy; should not be needed
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); // We don't want the old OpenGL 

	// Open a window and create its OpenGL context
	window = glfwCreateWindow( windowsize.width, windowsize.height, "BoardGL", NULL, NULL);
	if( window == NULL ){
		cerr << "Failed to open GLFW window. If you have an Intel GPU, they are not 3.3 compatible. Try the 2.1 version of the tutorials." << endl;
		glfwTerminate();
		return nullptr;
	}
	glfwMakeContextCurrent(window); // Initialize GLEW
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

	glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);
	glfwSetInputMode(window, GLFW_RAW_MOUSE_MOTION, GL_TRUE);

	glfwSetWindowSizeCallback(window, window_size_callback);

	return window;
}

glm::mat4 BoardGL::GetVPMatrix(glm::vec3 forward, glm::vec3 up)
{
	int winwidth, winheight;
	glfwGetWindowSize(window, &winwidth, &winheight);

	glm::mat4 CameraMatrix = glm::lookAt(
		cameraPosition,
		cameraPosition+forward,
		up
	);

	glm::mat4 projectionMatrix = glm::perspective(
		glm::radians(FoV),						// The vertical Field of View, in radians
		(float) winwidth / (float)winheight,	//Aspect ratio
		0.01f,									// Near clipping plane.
		100.0f									// Far clipping plane.
	);

	return projectionMatrix * CameraMatrix;
}

glm::vec3 BoardGL::GetDirection()
{
	// Direction : Spherical coordinates to Cartesian coordinates conversion
	double sinh, cosh;
	sincos(horizontalAngle, &sinh, &cosh);
	double sinv, cosv;
	sincos(verticalAngle, &sinv, &cosv);
	glm::vec3 direction(
		cosv * sinh,
		cosv * cosh,
		sinv
	);
	return direction;
}


glm::vec3 BoardGL::GetRightVector()
{
	// Right vector
	double sinh, cosh;
	sincos(horizontalAngle, &sinh, &cosh);
	glm::vec3 right = glm::vec3(
		cosh,
		-sinh,
		0
	);
	return right;
}

glm::mat4 Affine3DToGLM(cv::Affine3d Location)
{
	glm::mat4 outmat;
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			outmat[j][i] = Location.matrix(i,j);
		}
	}
	return outmat;
}


void BoardGL::HandleInputs()
{
	double currentTime = glfwGetTime();
	float deltaTime = float(currentTime - lastTime);
	int winwidth, winheight;
	glfwGetWindowSize(window, &winwidth, &winheight);

	if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT))
	{
		// Get mouse position
		double xpos, ypos;
		if (LookingAround)
		{
			glfwGetCursorPos(window, &xpos, &ypos);
		}
		else
		{
			LookingAround = true;
			xpos = winwidth/2.0;
			ypos = winheight/2.0;
			glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
		}
		
		glfwSetCursorPos(window, winwidth/2.0, winheight/2.0);
		// Compute new orientation
		double xdiff = winwidth/2.0 - xpos, ydiff = winheight/2.0 - ypos;
		horizontalAngle -= mouseSpeed * xdiff;
		verticalAngle 	+= mouseSpeed * ydiff;
		//cout << "X: " << xdiff << " Y :" << ydiff << " h: " << horizontalAngle << " v: " << verticalAngle << endl;
		verticalAngle = clamp<float>(verticalAngle, -M_PI_2, M_PI_2);
	}
	else
	{
		LookingAround = false;
		glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
	}
	

	

	glm::vec3 direction = GetDirection();
	glm::vec3 right = GetRightVector();

	float speed = 1;
	// Move forward
	if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS){
		cameraPosition += direction * deltaTime * speed;
	}
	// Move backward
	if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS){
		cameraPosition -= direction * deltaTime * speed;
	}
	// Strafe right
	if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS){
		cameraPosition += right * deltaTime * speed;
	}
	// Strafe left
	if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS){
		cameraPosition -= right * deltaTime * speed;
	}

	//cout << "pos: " << cameraPosition.x << " " << cameraPosition.y << " " << cameraPosition.z << endl;

	lastTime = currentTime;
}

void BoardGL::LoadModels()
{
	if (!MeshesLoaded)
	{
		//cout << "Loading meshes" << endl;
		robot.LoadFromFile("../assets/robot.obj");
		arena.LoadFromFile("../assets/board.obj", "../assets/boardtex.png");
		brio.LoadFromFile("../assets/BRIO.obj");
		axis.LoadFromFile("../assets/axis.obj");
		skybox.LoadFromFile("../assets/skybox.obj");
		robot.BindMesh();
		arena.BindMesh();
		brio.BindMesh();
		axis.BindMesh();
		skybox.BindMesh();
		MeshesLoaded = true;
	}
	
}
void BoardGL::LoadTags()
{
	if (TagsLoaded)
	{
		return;
	}
	glfwMakeContextCurrent(window);
	tag.LoadFromFile("../assets/tag.obj");
	tag.BindMesh();

	TagTextures.resize(100);
	auto& det = GetArucoDetector();
	auto& dict = det.getDictionary();
	for (int i = 0; i < 100; i++)
	{
		cv::Mat texture;
		cv::aruco::generateImageMarker(dict, i, 60, texture, 1);
		cv::cvtColor(texture, TagTextures[i].Texture, cv::COLOR_GRAY2BGR);
		//TagTextures[i].Texture = texture;
		TagTextures[i].valid = true;
		TagTextures[i].Bind();
	}
	TagsLoaded = true;
	glfwMakeContextCurrent(NULL);
}

void BoardGL::Start()
{
	//cout << "Creating OpenGL context" << endl;
	if (!HasInit)
	{
		GLInit();
	}
	
	

	cv::Size winsize(1280,720);

	if (!HasWindow)
	{
		window = GLCreateWindow(winsize);
		HasWindow = true;
	}
	// Create and compile our GLSL program from the shaders
	ShaderProgram.LoadShader(shaderfolder + "vertexshader.vs", shaderfolder + "fragmentshader.fs");

	if (!HasInit)
	{
		glGenVertexArrays(1, &VertexArrayID);
		glBindVertexArray(VertexArrayID);
		HasInit = true;
	}
	LoadModels();
	glfwMakeContextCurrent(NULL);

	//cout << "OpenGL init done!" << endl;
}

bool BoardGL::Tick(std::vector<ObjectData> data)
{
	glfwMakeContextCurrent(window);
	glfwSwapBuffers(window);
	glfwPollEvents();
	HandleInputs();
	glm::vec3 direction = GetDirection();
	glm::vec3 right = GetRightVector();

	// Up vector : perpendicular to both direction and right
	glm::vec3 up = glm::cross(right, direction);

	glm::mat4 VPMatrix = GetVPMatrix(direction, up);

	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glUseProgram(ShaderProgram.ProgramID);

	GLuint MatrixID = glGetUniformLocation(ShaderProgram.ProgramID, "MVP"); //projection matrix handle
	GLuint TextureSamplerID  = glGetUniformLocation(ShaderProgram.ProgramID, "TextureSampler"); //texture sampler handle
	glUniform1i(TextureSamplerID, 0); //set texture sampler to use texture 0
	GLuint ParameterID = glGetUniformLocation(ShaderProgram.ProgramID, "Parameters");
	GLuint ScaleID = glGetUniformLocation(ShaderProgram.ProgramID, "scale");


	glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &VPMatrix[0][0]);
	glUniform1f(ScaleID, 1);
	axis.Draw(ParameterID);
	skybox.Draw(ParameterID);

	for (int i = 0; i < data.size(); i++)
	{
		ObjectData &odata = data[i];
		glm::mat4 ObjectMatrix = Affine3DToGLM(odata.location);
		glm::mat4 MVPMatrix = VPMatrix * ObjectMatrix;

		glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVPMatrix[0][0]);
		glUniform1f(ScaleID, 1);
		switch (odata.identity.type)
		{
		case PacketType::Robot :
			robot.Draw(ParameterID);
			break;
		case PacketType::ReferenceAbsolute :
		case PacketType::ReferenceRelative :
			arena.Draw(ParameterID);
			break;
		case PacketType::Camera :
			axis.Draw(ParameterID);
			brio.Draw(ParameterID);
			break;
		case PacketType::Tag :
			{
				if (!TagsLoaded)
				{
					cerr << "WARNING Tried to display tags but tags aren't loaded" << endl;
					break;
				}
				
				float scalefactor = (float)odata.identity.metadata/1000.f;
				glUniform1f(ScaleID, scalefactor);
				TagTextures[odata.identity.numeral].Draw();
				tag.Draw(ParameterID, true);
			}
			break;
		
		default:
			break;
		}
	}

	bool IsDone = glfwGetKey(window, GLFW_KEY_ESCAPE ) != GLFW_PRESS && glfwWindowShouldClose(window) == 0;

	glfwMakeContextCurrent(NULL);

	return IsDone;
}

void BoardGL::runTest()
{
	Start();

	int winwidth, winheight;
	glfwGetWindowSize(window, &winwidth, &winheight);
	

	lastTime = glfwGetTime();

	do{
		HandleInputs();

		glm::vec3 direction = GetDirection();
		glm::vec3 right = GetRightVector();

		// Up vector : perpendicular to both direction and right
		glm::vec3 up = glm::cross(right, direction);

		// Clear the screen. It's not mentioned before Tutorial 02, but it can cause flickering, so it's there nonetheless.
		glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		glUseProgram(ShaderProgram.ProgramID);



		glm::mat4 MVPmatrix = GetVPMatrix(direction, up) /** model*/;

		// Get a handle for our "MVP" uniform
		// Only during the initialisation
		GLuint MatrixID = glGetUniformLocation(ShaderProgram.ProgramID, "MVP");
		
		// Send our transformation to the currently bound shader, in the "MVP" uniform
		// This is done in the main loop since each model will have a different MVP matrix (At least for the M part)
		glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVPmatrix[0][0]);

		robot.Draw();

		// Swap buffers
		glfwSwapBuffers(window);
		glfwPollEvents();

	} // Check if the ESC key was pressed or the window was closed
	while( glfwGetKey(window, GLFW_KEY_ESCAPE ) != GLFW_PRESS &&
		glfwWindowShouldClose(window) == 0 );
}

void BoardGL::InspectObject(TrackedObject* object)
{
	Start();

	LoadTags();

	vector<ObjectData> datas = object->ToObjectData(0);
	for (size_t i = 0; i < object->markers.size(); i++)
	{
		ArucoMarker &m = object->markers[i];
		ObjectData d;
		d.identity.numeral = m.number;
		d.identity.type = PacketType::Tag;
		d.identity.metadata = m.sideLength*1000;
		d.location = m.Pose;
		datas.push_back(d);
	}
	while (Tick(datas));
	
}