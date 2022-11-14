#include "TrackedObjects/TrackedObject.hpp"
#include "TrackedObjects/TrackerCube.hpp" // for the test env
#include <opencv2/core.hpp>
#include <opencv2/core/affine.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/imgproc.hpp>


#include "math3d.hpp"
#include "Cameras/Camera.hpp"
#include "GlobalConf.hpp"
#include "data/CameraView.hpp"
#include "visualisation/BoardViz2D.hpp"

using namespace cv;
using namespace std;

vector<Point3d> ArucoMarker::GetObjectPointsNoOffset(float SideLength)
{
	float sql2 = SideLength*0.5;
	return {
		Point3d(-sql2, sql2, 0.0),
		Point3d(sql2, sql2, 0.0),
		Point3d(sql2, -sql2, 0.0),
		Point3d(-sql2, -sql2, 0.0)
	};
}

vector<Point3d>& ArucoMarker::GetObjectPointsNoOffset()
{
	return ObjectPointsNoOffset;
}

void ArucoMarker::DisplayMarker(viz::Viz3d* visualizer, Affine3d RootLocation, String rootName)
{
	Affine3d Location = RootLocation * Pose;
	viz::WImage3D widget(GetArucoImage(number), Size2d(sideLength, sideLength));
	visualizer->showWidget(rootName + "/" + to_string(number), widget, Location * Affine3d(ImageToWorld()));
	visualizer->showWidget(rootName + "/" + to_string(number) + "/axis", viz::WCoordinateSystem(0.01), Location);
}

bool TrackedObject::SetLocation(Affine3d InLocation)
{
	Location = InLocation;
	return true;
}

Affine3d TrackedObject::GetLocation()
{
	return Location;
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

bool TrackedObject::FindTag(int MarkerID, ArucoMarker& Marker, Affine3d& TransformToMarker)
{
	for (int i = 0; i < markers.size(); i++)
	{
		if (markers[i].number == MarkerID)
		{
			Marker = markers[i];
			TransformToMarker = Affine3d::Identity();
			return true;
		}
	}
	for (int i = 0; i < childs.size(); i++)
	{
		if(childs[i]->FindTag(MarkerID, Marker, TransformToMarker))
		{
			TransformToMarker = childs[i]->Location * TransformToMarker;
			return true;
		}
	}
	return false;
}

void TrackedObject::GetObjectPoints(vector<vector<Point3d>>& MarkerCorners, vector<int>& MarkerIDs, Affine3d rootTransform, vector<int> filter)
{
	for (int i = 0; i < markers.size(); i++)
	{
		ArucoMarker& marker = markers[i];
		//If filter is not empty and the number wasn't found in he filter
		if (filter.size() != 0 && std::find(filter.begin(), filter.end(), marker.number) == filter.end())
		{
			continue;
		}
		vector<Point3d> cornerslocal = marker.GetObjectPointsNoOffset();
		MarkerIDs.push_back(marker.number);
		vector<Point3d> cornersworld;
		for (int i = 0; i < cornerslocal.size(); i++)
		{
			cornersworld.push_back(rootTransform * (marker.Pose * cornerslocal[i]));
		}
		MarkerCorners.push_back(cornersworld);
	}
	for (int i = 0; i < childs.size(); i++)
	{
		TrackedObject* child = childs[i];
		child->GetObjectPoints(MarkerCorners, MarkerIDs, rootTransform * child->Location, filter);
	}
}

Affine3d TrackedObject::GetObjectTransform(CameraArucoData& CameraData, float& Surface)
{
	vector<vector<Point3d>> MarkerCorners3D, objectPoints; 
	vector<int> MarkerIDs3D, IDs;
	vector<vector<Point2f>> imagePoints;
	GetObjectPoints(MarkerCorners3D, MarkerIDs3D, Affine3d::Identity(), CameraData.TagIDs);
	Surface = 0;
	//Keep only tags which have a 3D counterpart in this object, and link them together
	for (int index2d = 0; index2d < CameraData.TagIDs.size(); index2d++)
	{
		int id = CameraData.TagIDs[index2d];
		auto foundloc = std::find(MarkerIDs3D.begin(), MarkerIDs3D.end(), id);
		if (foundloc == MarkerIDs3D.end()) // not found
		{
			continue;
		}
		int index3d = foundloc - MarkerIDs3D.begin();
		objectPoints.push_back(MarkerCorners3D[index3d]);
		imagePoints.push_back(CameraData.TagCorners[index2d]);
		IDs.push_back(id);
		Surface += contourArea(CameraData.TagCorners[index2d], false);
	}

	if (IDs.size() <=0)
	{
		Surface = 0;
		return Affine3d::Identity();
	}
	Affine3d localTransform;
	if (IDs.size() == 1)
	{
		ArucoMarker marker; Affine3d transform;
		if (!FindTag(IDs[0], marker, transform))
		{
			return Affine3d::Identity();
		}
		Affine3d objectToMarker = transform * marker.Pose;
		Affine3d CameraToMarker = GetTagTransform(marker.sideLength, imagePoints[0], CameraData.CameraMatrix, CameraData.DistanceCoefficients);
		localTransform = CameraToMarker * objectToMarker.inv();
	}
	else
	{
		Mat rvec = Mat::zeros(3, 1, CV_64F), tvec = Mat::zeros(3, 1, CV_64F);
		vector<Point3d> flatobj;
		vector<Point2f> flatimg;
		for (int i = 0; i < IDs.size(); i++)
		{
			for (int j = 0; j < 4; j++)
			{
				flatobj.push_back(objectPoints[i][j]);
				flatimg.push_back(imagePoints[i][j]);
			}
		}
		
		solvePnP(flatobj, flatimg, CameraData.CameraMatrix, CameraData.DistanceCoefficients, rvec, tvec, false, SOLVEPNP_SQPNP);
		Matx33d rotationMatrix; //Matrice de rotation Camera -> Tag
		Rodrigues(rvec, rotationMatrix);
		localTransform = Affine3d(rotationMatrix, tvec);
	}

	return localTransform;

	
}

void TrackedObject::DisplayRecursive2D(BoardViz2D* visualizer, Affine3d RootLocation, String rootName)
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

vector<ObjectData> TrackedObject::ToObjectData(int BaseNumeral)
{
	cerr << "WARNING: Call to base TrackedObject::ToObjectData function. That function is uninplemented. Override it." <<endl;
	return {};
}

Affine3d GetTagTransform(float SideLength, std::vector<Point2f> Corners, Mat& CameraMatrix, Mat& DistanceCoefficients)
{
	Mat rvec, tvec;
	Mat distCoeffs = Mat::zeros(4,1, CV_64F);
	solvePnP(ArucoMarker::GetObjectPointsNoOffset(SideLength), Corners, 
		CameraMatrix, DistanceCoefficients, rvec, tvec, false, SOLVEPNP_IPPE_SQUARE);
	Matx33d rotationMatrix; //Matrice de rotation Camera -> Tag
	Rodrigues(rvec, rotationMatrix);
	return Affine3d(rotationMatrix, tvec);
}

Affine3d GetTagTransform(float SideLength, std::vector<Point2f> Corners, ArucoCamera* Cam)
{
	Mat rvec, tvec;
	Mat CamMatrix, distCoeffs;
	Cam->GetCameraSettingsAfterUndistortion(CamMatrix, distCoeffs);
	return GetTagTransform(SideLength, Corners, CamMatrix, distCoeffs);
}

Affine3d GetTransformRelativeToTag(ArucoMarker& Tag, std::vector<Point2f> Corners, ArucoCamera* Cam)
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

