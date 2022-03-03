#include "trackedobject.hpp"
#include <opencv2/core.hpp>
#include <opencv2/core/affine.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/viz.hpp>

#include "math3d.hpp"
#include "Camera.hpp"
#include "GlobalConf.hpp"

extern ArucoMarker center(0.1, 42, Affine3d(Vec3d::all(0), Vec3d(0, -0.25, 0)));

void ArucoMarker::DisplayMarker(viz::Viz3d* visualizer, Affine3d RootLocation)
{
	Affine3d Location = RootLocation * Pose;
	viz::WImage3D widget(GetArucoImage(number), Size2d(sideLength, sideLength));
	char buff[128];
	snprintf(buff, 127, "%p marker", this);
	visualizer->showWidget(buff, widget, Location * Affine3d(ImageToWorld()));
	char buff2[128];
	snprintf(buff2, 127, "%p coord", this);
	visualizer->showWidget(buff2, viz::WCoordinateSystem(0.01), Location);
}

Affine3d trackedobject::ResolveLocation(vector<ArucoView> views)
{
	if (views.size() == 1)
	{
		return views[0].CameraPosition * views[0].MarkerPosition;
	}
	
}

void trackedobject::DisplayRecursive(viz::Viz3d* visualizer, Affine3d RootLocation)
{
	Affine3d worldlocation = RootLocation * Location;
	for (int i = 0; i < markers.size(); i++)
	{
		markers[i].DisplayMarker(visualizer, worldlocation);
	}
	
	for (int i = 0; i < childs.size(); i++)
	{
		childs[i]->DisplayRecursive(visualizer, worldlocation);
	}
	
}

TrackerCube::TrackerCube(vector<int> MarkerIdx, float MarkerSize, Point3d CubeSize)
{
	Unique = false;
	vector<Point3d> Locations = {Point3d(CubeSize.x,0,0), Point3d(0,CubeSize.y,0), Point3d(-CubeSize.x,0,0), Point3d(0,-CubeSize.y,0)};
	for (int i = 0; i < 4; i++)
	{
		Matx33d markerrot = MakeRotationFromZY(Locations[i], Vec3d(0,0,1));
		Affine3d markertransform = Affine3d(markerrot, Locations[i]);
		ArucoMarker marker(MarkerSize, MarkerIdx[i], markertransform);
		markers.push_back(marker);
	}
	
}

TrackerCube::~TrackerCube()
{
}

vector<Point2f> ReorderMarkerCorners(vector<Point2f> Corners)
{
	vector<Point2f> newCorners{Corners[0], Corners[1], Corners[2], Corners[3]};
	return newCorners;
}

Affine3d GetTagTransform(ArucoMarker& Tag, std::vector<Point2f> Corners, Camera* Cam)
{
	Mat rvec, tvec;
	solvePnP(Tag.GetObjectPointsNoOffset(), ReorderMarkerCorners(Corners), Cam->CameraMatrix, Cam->distanceCoeffs, rvec, tvec, false, SOLVEPNP_IPPE_SQUARE);
	Matx33d rotationMatrix; //Matrice de rotation Camera -> Tag
	Rodrigues(rvec, rotationMatrix);
	return Affine3d(rotationMatrix, tvec);
}

Affine3d GetTransformRelativeToTag(ArucoMarker& Tag, std::vector<Point2f> Corners, Camera* Cam)
{
	return Tag.Pose * GetTagTransform(Tag, Corners, Cam).inv();
}

void Tracker3DTest()
{
	viz::Viz3d env("3D tracker position check");
	viz::WCoordinateSystem coords(0.1);
	env.showWidget("coordinate", coords);
	TrackerCube cube({50, 51, 52, 54}, 0.06, Point3d(0.05, 0.05, 0));
	cube.DisplayRecursive(&env, Affine3d::Identity());
	env.spin();
}

