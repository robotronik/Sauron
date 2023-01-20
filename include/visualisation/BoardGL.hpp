#pragma once

#include <vector>

#include <glm/glm.hpp>

#include "TrackedObjects/ObjectIdentity.hpp"

#include "visualisation/openGL/Mesh.hpp"
#include "visualisation/openGL/Shader.hpp"

class GLFWwindow;
class TrackedObject;

class BoardGL
{
private:
	bool HasWindow = false;
	GLFWwindow* window;
	static bool HasInit;
	static GLuint VertexArrayID;
	static Shader ShaderProgram;

	static bool MeshesLoaded, TagsLoaded;
	static Mesh robot, arena, puck, brio, axis, tag;
	static std::vector<Texture> TagTextures;

	void GLInit();
	GLFWwindow* GLCreateWindow(cv::Size windowsize);

	glm::mat4 GetVPMatrix(glm::vec3 forward, glm::vec3 up);
public:

	bool LookingAround = false;
	float FoV = 60.f, mouseSpeed = 0.002f;
	float horizontalAngle = 0.f, verticalAngle = -M_PI_2;
	glm::vec3 cameraPosition = glm::vec3(0,0,2);

	double lastTime;

	glm::vec3 GetDirection();

	glm::vec3 GetRightVector();

	void HandleInputs();

	void LoadModels();
	void LoadTags();
	
	void Start();

	bool Tick(std::vector<ObjectData> data);

	void runTest();

	void InspectObject(TrackedObject* object);

};
