#include "TrackedObjects/TrackedObject.hpp"
#include "TrackedObjects/TrackerCube.hpp" // for the test env
#include <opencv2/core.hpp>
#include <opencv2/core/affine.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/viz.hpp>


#include "math3d.hpp"
#include "Camera.hpp"
#include "GlobalConf.hpp"
#include "data/SerialPacket.hpp"
#include "data/CameraView.hpp"
#include "visualisation/BoardViz2D.hpp"

ArucoMarker center(0.1, 42, Affine3d(Vec3d::all(0), Vec3d(0, -0.25, 0)));

void ArucoMarker::DisplayMarker(viz::Viz3d* visualizer, Affine3d RootLocation, String rootName)
{
	Affine3d Location = RootLocation * Pose;
	viz::WImage3D widget(GetArucoImage(number), Size2d(sideLength, sideLength));
	visualizer->showWidget(rootName + "/" + to_string(number), widget, Location * Affine3d(ImageToWorld()));
	visualizer->showWidget(rootName + "/" + to_string(number) + "/axis", viz::WCoordinateSystem(0.01), Location);
}

Affine3d TrackedObject::ResolveLocation(vector<Affine3d>& Cameras, vector<CameraView>& views)
{
	vector<Affine3d> positions;
	vector<Vec3d> CameraRays;
	vector<int> bestViews; //each element contains the index of a view which is the best for the camera
	positions.resize(views.size());
	CameraRays.resize(views.size());

	for (int ViewIdx = 0; ViewIdx < views.size(); ViewIdx++)
	{
		for (int MarkerIdx = 0; MarkerIdx < markers.size(); MarkerIdx++)
		{
			if (markers[MarkerIdx].number == views[ViewIdx].TagID)
			{
				Affine3d MarkerWorld = Cameras[views[ViewIdx].Camera] * views[ViewIdx].TagTransform;
				Affine3d OriginWorld = MarkerWorld * markers[MarkerIdx].Pose.inv();
				positions[ViewIdx] = OriginWorld;
				Vec3d ray = OriginWorld.translation() - Cameras[views[ViewIdx].Camera].translation();
				double raylength = sqrt(ray.ddot(ray));
				Vec3d raydir = ray / raylength;
				CameraRays[ViewIdx] = ray;
				views[ViewIdx].score = MarkerWorld.rotation().col(2).ddot(-raydir);
				bool HasCameraInBestViews = false;
				for (int k = 0; k < bestViews.size(); k++)
				{
					if (views[bestViews[k]].Camera != views[ViewIdx].Camera)
					{
						continue;
					}
					if (views[bestViews[k]].score < views[ViewIdx].score)
					{
						bestViews[k] = ViewIdx;
					}
					HasCameraInBestViews = true;
					break;
				}
				if (!HasCameraInBestViews)
				{
					bestViews.push_back(ViewIdx);
				}
				break;
			}
		}
	}
	/*vector<Affine3d> ChildLocations;
	for (int i = 0; i < childs.size(); i++)
	{
		Affine3d ChildWorld = childs[i]->ResolveLocation(Cameras, views);
		ChildLocations.push_back(childs[i]->Location * ChildWorld);
	}*/
	if (bestViews.size() >= 2) //more than one camera sees the object
	{
		int best =-1;
		int secondbest = -1;
		for (int i = 0; i < bestViews.size(); i++)
		{
			if (best==-1)
			{
				best=i;
			}
			else if (views[bestViews[i]].score > views[bestViews[best]].score)
			{
				secondbest = best;
				best = i;
			}
			else if (secondbest ==-1)
			{
				secondbest = i;
			}
			else if (views[bestViews[i]].score > views[bestViews[secondbest]].score)
			{
				secondbest = i;
			}
		}
		int bestidx = bestViews[best];
		int secondbestidx = bestViews[secondbest];
		Vec3d bestpoint, secondbestpoint;
		ClosestPointsOnTwoLine(positions[bestidx].translation(), CameraRays[bestidx], 
		positions[secondbestidx].translation(), CameraRays[secondbestidx], bestpoint, secondbestpoint);

		Affine3d theend = Affine3d(positions[bestidx].rotation(), (bestpoint + secondbestpoint)/2);
		Location = theend;
	}
	else
	{
		/*cout << "Only one camera sees "<< Name << " so positioning using aruco #" 
		<< views[bestViews[0]].TagID << " (score " << views[bestViews[0]].score << ")" << endl;*/
		Location = positions[bestViews[0]];
	}
	return Location;
}

void TrackedObject::DisplayRecursive2D(BoardViz2D visualizer, Affine3d RootLocation, String rootName)
{
	Affine3d worldlocation = RootLocation * Location;

	for (int i = 0; i < childs.size(); i++)
	{
		childs[i]->DisplayRecursive2D(visualizer, worldlocation, rootName + "/" + Name);
	}
}

void TrackedObject::DisplayRecursive(viz::Viz3d* visualizer, Affine3d RootLocation, String rootName)
{
	Affine3d worldlocation = RootLocation * Location;
	for (int i = 0; i < markers.size(); i++)
	{
		markers[i].DisplayMarker(visualizer, worldlocation, rootName + "/" + Name);
	}
	
	for (int i = 0; i < childs.size(); i++)
	{
		childs[i]->DisplayRecursive(visualizer, worldlocation, rootName + "/" + Name);
	}
	
}

vector<PositionPacket> TrackedObject::ToPacket(int BaseNumeral)
{
	cerr << "WARNING: Call to base TrackedObject::ToPacket function. That function is uninplemented. Override it." <<endl;
	return {};
}

vector<Point2f> ReorderMarkerCorners(vector<Point2f> Corners)
{
	vector<Point2f> newCorners{Corners[0], Corners[1], Corners[2], Corners[3]};
	return newCorners;
}

Affine3d GetTagTransform(float SideLength, std::vector<Point2f> Corners, Mat& CameraMatrix, Mat& DistanceCoefficients)
{
	Mat rvec, tvec;
	Mat distCoeffs = Mat::zeros(4,1, CV_64F);
	solvePnP(ArucoMarker::GetObjectPointsNoOffset(SideLength), ReorderMarkerCorners(Corners), 
		CameraMatrix, DistanceCoefficients, rvec, tvec, false, SOLVEPNP_IPPE_SQUARE);
	Matx33d rotationMatrix; //Matrice de rotation Camera -> Tag
	Rodrigues(rvec, rotationMatrix);
	return Affine3d(rotationMatrix, tvec);
}

Affine3d GetTagTransform(float SideLength, std::vector<Point2f> Corners, Camera* Cam)
{
	Mat rvec, tvec;
	Mat CamMatrix, distCoeffs;
	Cam->GetCameraSettingsAfterUndistortion(CamMatrix, distCoeffs);
	return GetTagTransform(SideLength, Corners, CamMatrix, distCoeffs);
}

Affine3d GetTransformRelativeToTag(ArucoMarker& Tag, std::vector<Point2f> Corners, Camera* Cam)
{
	return Tag.Pose * GetTagTransform(Tag.sideLength, Corners, Cam).inv();
}

void Tracker3DTest()
{
	viz::Viz3d env("3D tracker position check");
	viz::WCoordinateSystem coords(0.1);
	env.showWidget("coordinate", coords);
	TrackerCube* cube = new TrackerCube({51, 52, 54, 55}, 0.06, Point3d(0.0952, 0.0952, 0), "Cube");
	cube->DisplayRecursive(&env, Affine3d::Identity(), String());
	env.spin();
}

