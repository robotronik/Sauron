#pragma once

#include <vector>

#include <glm/glm.hpp>

#include "TrackedObjects/ObjectIdentity.hpp"

#include "visualisation/openGL/Mesh.hpp"

class GLFWwindow;

class BoardGL
{
private:
    GLFWwindow* window;
    GLuint programID, VertexArrayID;

    Mesh robot, arena, puck, brio, axis;

    void GLInit();
    GLFWwindow* GLCreateWindow(cv::Size windowsize);
public:

    bool LookingAround = false;
	float FoV = 60.f, mouseSpeed = 0.002f;
    float horizontalAngle = 0.f, verticalAngle = 0.f;
    glm::vec3 cameraPosition = glm::vec3(1,0,0);

    double lastTime;

    glm::vec3 GetDirection();

    glm::vec3 GetRightVector();

    void HandleInputs();

    void Start();

    bool Tick(std::vector<ObjectData> data);

    void runTest();

};
