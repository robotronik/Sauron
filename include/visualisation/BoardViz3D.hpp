#pragma once

#ifdef WITH_VTK
#include <opencv2/core.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/core/affine.hpp>

#include "TrackedObjects/ObjectIdentity.hpp"

class Camera;

//A 3D visualiser wrapper bsed on VTK
//Deprecated
class BoardViz3D
{
private:
	static bool MeshesLoaded;
	static cv::viz::Mesh BoardMesh, RobotMesh, CameraMesh;
	static cv::Mat BoardMat;
	cv::viz::Viz3d *visualizer;
public:
	BoardViz3D(cv::viz::Viz3d *InVisualizer);
	~BoardViz3D();

	static void LoadMeshes();

	static void SetupTerrain(cv::viz::Viz3d& Visualizer);

	static void SetupRobot(cv::viz::Viz3d& Visualizer);

	static void ShowCamera(cv::viz::Viz3d& Visualizer, Camera* Camera, int BufferIdx, cv::Affine3d Pose, cv::viz::Color color);

	static cv::viz::Mesh GetBoardMesh();
	static cv::viz::Mesh GetRobotMesh();
	static cv::Mat GetBoardMat();

	static cv::Affine3d ImageToWorld();

	void DisplayData(std::vector<ObjectData> &objects);
};

void TestViz3D();
#endif