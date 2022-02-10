#pragma once

#include <opencv2/core.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/core/affine.hpp>
#include "position.hpp"
using namespace cv;


class Camera;

class BoardViz3D
{
private:
	static bool MeshesLoaded;
	static viz::Mesh BoardMesh, RobotMesh;
	static Mat BoardMat;
public:
	BoardViz3D();
	~BoardViz3D();

	static void LoadMeshes();

	static void SetupTerrain(viz::Viz3d& Visualizer);

	static void ShowCamera(viz::Viz3d& Visualizer, Camera* Camera, int BufferIdx, Affine3d Pose);

	static viz::Mesh GetBoardMesh();
	static viz::Mesh GetRobotMesh();
	static Mat GetBoardMat();

	static Affine3d ImageToWorld();
};

void TestViz3D();