#include "visualisation/BoardGL.hpp"

#include <iostream>
#include <fstream>

#include <opencv2/core.hpp>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/transform.hpp>
#include <assimp/Importer.hpp>

#include "visualisation/openGL/Mesh.hpp"

using namespace std;

bool BoardGL::MeshesLoaded = false;
Mesh BoardGL::robot, BoardGL::arena, BoardGL::axis, BoardGL::brio, BoardGL::puck;

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
	glfwWindowHint(GLFW_SAMPLES, 1); // 4x antialiasing
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3); // We want OpenGL 3.3
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // To make MacOS happy; should not be needed
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); // We don't want the old OpenGL 

	// Open a window and create its OpenGL context
	GLFWwindow* window; // (In the accompanying source code, this variable is global for simplicity)
	window = glfwCreateWindow( windowsize.width, windowsize.height, "BoardGL", NULL, NULL);
	if( window == NULL ){
		cerr << "Failed to open GLFW window. If you have an Intel GPU, they are not 3.3 compatible. Try the 2.1 version of the tutorials." << endl;
		glfwTerminate();
		return nullptr;
	}
	glfwMakeContextCurrent(window); // Initialize GLEW
	glewExperimental=true; // Needed in core profile
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

GLuint GLLoadShaders(string vertex_file_path, string fragment_file_path){

	// Create the shaders
	GLuint VertexShaderID = glCreateShader(GL_VERTEX_SHADER);
	GLuint FragmentShaderID = glCreateShader(GL_FRAGMENT_SHADER);

	// Read the Vertex Shader code from the file
	std::string VertexShaderCode;
	std::ifstream VertexShaderStream(vertex_file_path, std::ios::in);
	if(VertexShaderStream.is_open()){
		std::stringstream sstr;
		sstr << VertexShaderStream.rdbuf();
		VertexShaderCode = sstr.str();
		VertexShaderStream.close();
	}else{
		cout << "Impossible to open " << vertex_file_path << ". Are you in the right directory ? Don't forget to read the FAQ !" << endl;
		getchar();
		return 0;
	}

	// Read the Fragment Shader code from the file
	std::string FragmentShaderCode;
	std::ifstream FragmentShaderStream(fragment_file_path, std::ios::in);
	if(FragmentShaderStream.is_open()){
		std::stringstream sstr;
		sstr << FragmentShaderStream.rdbuf();
		FragmentShaderCode = sstr.str();
		FragmentShaderStream.close();
	}

	GLint Result = GL_FALSE;
	int InfoLogLength;

	// Compile Vertex Shader
	cout << "Compiling shader : " << vertex_file_path << endl;
	char const * VertexSourcePointer = VertexShaderCode.c_str();
	glShaderSource(VertexShaderID, 1, &VertexSourcePointer , NULL);
	glCompileShader(VertexShaderID);

	// Check Vertex Shader
	glGetShaderiv(VertexShaderID, GL_COMPILE_STATUS, &Result);
	glGetShaderiv(VertexShaderID, GL_INFO_LOG_LENGTH, &InfoLogLength);
	if ( InfoLogLength > 0 ){
		std::vector<char> VertexShaderErrorMessage(InfoLogLength+1);
		glGetShaderInfoLog(VertexShaderID, InfoLogLength, NULL, &VertexShaderErrorMessage[0]);
		printf("%s\n", &VertexShaderErrorMessage[0]);
	}

	// Compile Fragment Shader
	cout << "Compiling shader : " << fragment_file_path << endl;
	char const * FragmentSourcePointer = FragmentShaderCode.c_str();
	glShaderSource(FragmentShaderID, 1, &FragmentSourcePointer , NULL);
	glCompileShader(FragmentShaderID);

	// Check Fragment Shader
	glGetShaderiv(FragmentShaderID, GL_COMPILE_STATUS, &Result);
	glGetShaderiv(FragmentShaderID, GL_INFO_LOG_LENGTH, &InfoLogLength);
	if ( InfoLogLength > 0 ){
		std::vector<char> FragmentShaderErrorMessage(InfoLogLength+1);
		glGetShaderInfoLog(FragmentShaderID, InfoLogLength, NULL, &FragmentShaderErrorMessage[0]);
		printf("%s\n", &FragmentShaderErrorMessage[0]);
	}

	// Link the program
	cout << "Linking program" << endl;
	GLuint ProgramID = glCreateProgram();
	glAttachShader(ProgramID, VertexShaderID);
	glAttachShader(ProgramID, FragmentShaderID);
	glLinkProgram(ProgramID);

	// Check the program
	glGetProgramiv(ProgramID, GL_LINK_STATUS, &Result);
	glGetProgramiv(ProgramID, GL_INFO_LOG_LENGTH, &InfoLogLength);
	if ( InfoLogLength > 0 ){
		std::vector<char> ProgramErrorMessage(InfoLogLength+1);
		glGetProgramInfoLog(ProgramID, InfoLogLength, NULL, &ProgramErrorMessage[0]);
		cout << "Error : " << &ProgramErrorMessage[0] << endl;
	}
	
	glDetachShader(ProgramID, VertexShaderID);
	glDetachShader(ProgramID, FragmentShaderID);
	
	glDeleteShader(VertexShaderID);
	glDeleteShader(FragmentShaderID);
	

	return ProgramID;
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

void BoardGL::Start()
{
	if (!MeshesLoaded)
	{
		cout << "Loading meshes" << endl;
		robot.LoadFromFile("../assets/robot.obj");
		arena.LoadFromFile("../assets/board.obj", "../assets/boardtex.png");
		brio.LoadFromFile("../assets/BRIO.obj");
		axis.LoadFromFile("../assets/axis.obj");
		MeshesLoaded = true;
	}
	
	cout << "Creating OpenGL context" << endl;
	GLInit();

	cv::Size winsize(1280,720);

	window = GLCreateWindow(winsize);

	// Create and compile our GLSL program from the shaders
	programID = GLLoadShaders( shaderfolder + "vertexshader.vs", shaderfolder + "fragmentshader.fs" );

	glGenVertexArrays(1, &VertexArrayID);
	glBindVertexArray(VertexArrayID);

	robot.BindMesh();
	arena.BindMesh();
	brio.BindMesh();
	axis.BindMesh();
	cout << "OpenGL init done!" << endl;
}

bool BoardGL::Tick(std::vector<ObjectData> data)
{
	glfwSwapBuffers(window);
	glfwPollEvents();
	HandleInputs();
	glm::vec3 direction = GetDirection();
	glm::vec3 right = GetRightVector();

	// Up vector : perpendicular to both direction and right
	glm::vec3 up = glm::cross(right, direction);

	int winwidth, winheight;
	glfwGetWindowSize(window, &winwidth, &winheight);

	glm::mat4 CameraMatrix = glm::lookAt(
		cameraPosition,
		cameraPosition+direction,
		up
	);

	glm::mat4 projectionMatrix = glm::perspective(
		glm::radians(FoV),						// The vertical Field of View, in radians
		(float) winwidth / (float)winheight,	//Aspect ratio
		0.01f,									// Near clipping plane.
		100.0f									// Far clipping plane.
	);

	glm::mat4 VPMatrix = projectionMatrix * CameraMatrix;

	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glUseProgram(programID);

	GLuint MatrixID = glGetUniformLocation(programID, "MVP"); //projection matrix handle
	GLuint TextureSamplerID  = glGetUniformLocation(programID, "TextureSampler"); //texture sampler handle
	glUniform1i(TextureSamplerID, 0); //set texture sampler to use texture 0
	GLuint ParameterID = glGetUniformLocation(programID, "Parameters");


	glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &VPMatrix[0][0]);
	axis.Draw();

	for (int i = 0; i < data.size(); i++)
	{
		ObjectData &odata = data[i];
		glm::mat4 ObjectMatrix = Affine3DToGLM(odata.location);
		glm::mat4 MVPMatrix = VPMatrix * ObjectMatrix;

		glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVPMatrix[0][0]);
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
		
		default:
			break;
		}
	}

	return glfwGetKey(window, GLFW_KEY_ESCAPE ) != GLFW_PRESS && glfwWindowShouldClose(window) == 0;
}

void BoardGL::runTest()
{
	Start();

	int winwidth, winheight;
	glfwGetWindowSize(window, &winwidth, &winheight);

	// Generates a really hard-to-read matrix, but a normal, standard 4x4 matrix nonetheless
	glm::mat4 projectionMatrix = glm::perspective(
		glm::radians(FoV),	// The vertical Field of View, in radians: the amount of "zoom". Think "camera lens". Usually between 90° (extra wide) and 30° (quite zoomed in)
		(float) winwidth / (float)winheight,		// Aspect Ratio. Depends on the size of your window. Notice that 4/3 == 800/600 == 1280/960, sounds familiar ?
		0.01f,				// Near clipping plane. Keep as big as possible, or you'll get precision issues.
		100.0f				// Far clipping plane. Keep as little as possible.
	);

	glm::mat4 CameraMatrix;

	lastTime = glfwGetTime();

	do{
		HandleInputs();

		glm::vec3 direction = GetDirection();
		glm::vec3 right = GetRightVector();

		// Up vector : perpendicular to both direction and right
		glm::vec3 up = glm::cross(right, direction);

		CameraMatrix = glm::lookAt(
			cameraPosition,           // Camera is here
			cameraPosition+direction, // and looks here : at the same position, plus "direction"
			up                  // Head is up (set to 0,-1,0 to look upside-down)
		);

		// Clear the screen. It's not mentioned before Tutorial 02, but it can cause flickering, so it's there nonetheless.
		glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		glUseProgram(programID);



		glm::mat4 MVPmatrix = projectionMatrix * CameraMatrix /** model*/;

		// Get a handle for our "MVP" uniform
		// Only during the initialisation
		GLuint MatrixID = glGetUniformLocation(programID, "MVP");
		
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