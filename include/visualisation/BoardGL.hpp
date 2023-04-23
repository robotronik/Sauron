#pragma once

#include <vector>
#include <map>

#include <glm/glm.hpp>

#include "TrackedObjects/ObjectIdentity.hpp"

#include "visualisation/GLWindow.hpp"
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
class BoardGL : public GLWindow
{
private:
	GLuint VertexArrayID;
	Shader ShaderProgram;

	bool MeshesLoaded = false, TagsLoaded= false;
	std::map<MeshNames, Mesh> Meshes;
	std::vector<Texture> TagTextures;

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

	virtual void WindowSizeCallback(int width, int height) override;

};
