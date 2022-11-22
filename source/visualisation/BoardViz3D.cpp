#include "visualisation/BoardViz3D.hpp"

#include <string>

#include <opencv2/viz.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/affine.hpp>

#include "thirdparty/serialib.h"
#include "GlobalConf.hpp"
#include "data/Calibfile.hpp"
#include "data/CameraView.hpp"
#include "TrackedObjects/TrackedObject.hpp"
#include "Cameras/Camera.hpp"
#include "math3d.hpp"

using namespace cv;
using namespace std;

const String assetpath = "../assets/";

bool BoardViz3D::MeshesLoaded;
viz::Mesh BoardViz3D::BoardMesh, BoardViz3D::RobotMesh, BoardViz3D::CameraMesh;
Mat BoardViz3D::BoardMat;

BoardViz3D::BoardViz3D(cv::viz::Viz3d *InVisualizer)
	:visualizer(InVisualizer)
{
	LoadMeshes();
}

BoardViz3D::~BoardViz3D()
{
}

void BoardViz3D::LoadMeshes()
{
	if (!MeshesLoaded)
	{
		BoardMesh = viz::Mesh::load(assetpath + "board.obj", viz::Mesh::LOAD_OBJ);
		RobotMesh = viz::Mesh::load(assetpath + "robot.obj", viz::Mesh::LOAD_OBJ);
		CameraMesh = viz::Mesh::load(assetpath + "BRIO.obj", viz::Mesh::LOAD_OBJ);
		BoardMat = imread(assetpath + "VisuelTable.png");
		MeshesLoaded = true;
	}
	
}

void BoardViz3D::SetupTerrain(viz::Viz3d& Visualizer)
{
	Visualizer.showWidget("axis", viz::WCoordinateSystem(0.25));
	Visualizer.showWidget("BoardMesh", viz::WMesh(BoardMesh));
	viz::WImage3D tableImg(BoardMat, Size2d(3, 2));
	//Visualizer.setRenderingProperty("BoardMesh", viz::REPRESENTATION, viz::REPRESENTATION_WIREFRAME);
	Visualizer.showWidget("BoardMat", tableImg, ImageToWorld());
	//Visualizer.setFullScreen(true);
	Visualizer.setViewerPose(Affine3d(MakeRotationFromZX(Vec3d(0,0,-1), Vec3d(1,0,0)), Vec3d(0,0,4)));
}

void BoardViz3D::SetupRobot(viz::Viz3d& Visualizer)
{
	Visualizer.showWidget("axis", viz::WCoordinateSystem(0.25));
	Visualizer.showWidget("Robot", viz::WMesh(RobotMesh));
	Affine3d CamTransform(MakeRotationFromZY(Vec3d(1,0,0), Vec3d(0,0,-1)), Vec3d(0.06375, 0, 0.330));
	Visualizer.showWidget("CameraFront", viz::WMesh(CameraMesh), CamTransform);
	Mat CamMatrix, distCoeffs;
	readCameraParameters(String("../calibration/Brio"), CamMatrix, distCoeffs, GetFrameSize());
	Visualizer.showWidget("CamFrustrum", viz::WCameraPosition((Matx33d)CamMatrix, 0.2), CamTransform);
	//Visualizer.spin();
}

void BoardViz3D::ShowCamera(viz::Viz3d& Visualizer, Camera* Camera, int BufferIdx, Affine3d Pose, viz::Color color)
{
	viz::WCameraPosition CamWidget;
	CameraSettings stg = Camera->GetCameraSettings();
	Mat CameraMatrix = stg.CameraMatrix;
	Size2d FOV = GetCameraFOV(stg.Resolution, stg.CameraMatrix);
	double maxfov = 160.0/180.0*M_PI;
	if (FOV.width >= maxfov || FOV.height >= maxfov)
	{
		CamWidget = viz::WCameraPosition(Vec2d(170,170), 0.2, color);
	}
	else
	{
		//CamWidget = viz::WCameraPosition(Vec2d(Camera->CameraMatrix.at<double>(0,0), Camera->CameraMatrix.at<double>(1,1)), 0.2, color);
		CamWidget = viz::WCameraPosition((Matx33d)(Camera->GetCameraSettings().CameraMatrix), 0.2, color);
	}
	Visualizer.showWidget(Camera->GetCameraSettings().DeviceInfo.device_paths[0], CamWidget, Pose);
	Visualizer.showWidget(Camera->GetCameraSettings().DeviceInfo.device_paths[0] + "axis", viz::WCoordinateSystem(0.1), Pose);
	return;
}

viz::Mesh BoardViz3D::GetBoardMesh()
{
	return BoardMesh;
}

viz::Mesh BoardViz3D::GetRobotMesh()
{
	return RobotMesh;
}

Mat BoardViz3D::GetBoardMat()
{
	return BoardMat;
}

Affine3d BoardViz3D::ImageToWorld()
{
	return Affine3d(Matx33d(1,0,0, 0,-1,0, 0,0,1), Vec3d::all(0));
}

void BoardViz3D::DisplayData(std::vector<ObjectData> &objects)
{
	for (int i = 0; i < objects.size(); i++)
	{
		ObjectData &object = objects[i];
		switch (object.identity.type)
		{
		case PacketType::Camera :
			{
				viz::WCameraPosition CamWidget = viz::WCameraPosition(Vec2d(170,170), 0.2, viz::Color::blue());
				visualizer->showWidget(string("cam") + to_string(object.identity.numeral), CamWidget, object.location);
				visualizer->showWidget(string("cam") + to_string(object.identity.numeral) + "axis", viz::WCoordinateSystem(0.1), object.location);
			}
			break;
		case PacketType::ReferenceAbsolute :
		case PacketType::ReferenceRelative :
			{
				visualizer->showWidget("BoardMesh", viz::WMesh(BoardMesh), object.location);
				viz::WImage3D tableImg(BoardMat, Size2d(3, 2));
				visualizer->showWidget("BoardMat", tableImg, object.location * ImageToWorld());
			}
			break;
		case PacketType::Robot :
			{
				visualizer->showWidget("axis", viz::WCoordinateSystem(0.25), object.location);
				visualizer->showWidget("Robot", viz::WMesh(RobotMesh), object.location);
			}
			break;
		case PacketType::Puck :
			break;
		
		default:
			break;
		}
	}
	
}

void TestViz3D()
{
	viz::Viz3d board("3D Board");
	BoardViz3D::SetupTerrain(board);
	board.spin();
	exit(EXIT_SUCCESS);
}