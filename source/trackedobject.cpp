#include "trackedobject.hpp"
#include <opencv2/core.hpp>
#include <opencv2/core/affine.hpp>
#include <opencv2/calib3d.hpp>

#include "Camera.hpp"

extern ArucoMarker center(0.1, 42, Affine3d(Vec3d::all(0), Vec3d(0, -0.25, 0)));

Affine3d trackedobject::ResolveLocation(vector<ArucoView> views)
{
	if (views.size() == 1)
	{
		return views[0].CameraPosition * views[0].MarkerPosition;
	}
	
}

TrackerCube::TrackerCube(vector<int> MarkerIdx, float MarkerSize, Point3d CubeSize)
{
	Unique = false;
	vector<Point3d> Locations = {Point3d(CubeSize.x,0,0), Point3d(0,CubeSize.y,0), Point3d(-CubeSize.x,0,0), Point3d(0,-CubeSize.y,0)};
	for (int i = 0; i < 4; i++)
	{
		Affine3d markertransform = Affine3d(Vec3d())
		ArucoMarker marker()
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

