#pragma once

#include <vector>
#include <map>

#include <glm/glm.hpp>

#include "TrackedObjects/ObjectIdentity.hpp"

#include "visualisation/openGL/Mesh.hpp"
#include "visualisation/openGL/Shader.hpp"

class GLFWwindow;
class TrackedObject;

enum class MeshNames
{
	robot,
	tag,
	arena,
	brio,
	skybox,
	axis,
	trackercube,
	toptracker
};
class BoardGL
{
private:
	bool HasWindow = false;
	GLFWwindow* window;
	static bool HasInit;
	static GLuint VertexArrayID;
	static Shader ShaderProgram;

	static bool MeshesLoaded, TagsLoaded;
	static std::unordered_map<MeshNames, Mesh> Meshes;
	static std::vector<Texture> TagTextures;

	void GLInit(); //Start OpenGL
	GLFWwindow* GLCreateWindow(cv::Size windowsize); //Create a window

	glm::mat4 GetVPMatrix(glm::vec3 forward, glm::vec3 up);
public:

	bool LookingAround = false; //Is left button pressed ?
	float FoV = 60.f, mouseSpeed = 0.002f;
	float horizontalAngle = 0.f, verticalAngle = -M_PI_2;
	glm::vec3 cameraPosition = glm::vec3(0,0,2);

	double lastTime;

	glm::vec3 GetDirection();

	glm::vec3 GetRightVector();

	void HandleInputs(); //Move camera and stuff

	void LoadModels();
	void LoadTags();
	
	void Start();

	bool Tick(std::vector<ObjectData> data); //Run display loop for these objects, returns false if exit was asked.

	void runTest();

	void InspectObject(TrackedObject* object); //Dsiplay this objects and it's tags

};
