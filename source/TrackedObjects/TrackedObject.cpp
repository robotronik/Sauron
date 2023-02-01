#include "TrackedObjects/TrackedObject.hpp"
#include "TrackedObjects/TrackerCube.hpp" // for the test env
#include <opencv2/core.hpp>
#include <opencv2/core/affine.hpp>
#include <opencv2/calib3d.hpp>
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

#ifdef WITH_VTK
void ArucoMarker::DisplayMarker(viz::Viz3d* visualizer, Affine3d RootLocation, String rootName)
{
	Affine3d Location = RootLocation * Pose;
	viz::WImage3D widget(GetArucoImage(number), Size2d(sideLength, sideLength));
	visualizer->showWidget(rootName + "/" + to_string(number), widget, Location * Affine3d(ImageToWorld()));
	visualizer->showWidget(rootName + "/" + to_string(number) + "/axis", viz::WCoordinateSystem(0.01), Location);
}
#endif

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
				float localscore = MarkerWorld.rotation().col(2).ddot(-raydir);
				views[ViewIdx].score = localscore;
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

float TrackedObject::GetSeenMarkers(const CameraArucoData& CameraData, vector<ArucoViewCameraLocal> &MarkersSeen, cv::Affine3d AccumulatedTransform)
{
	float surface = 0;
	for (int i = 0; i < markers.size(); i++)
	{
		for (int j = 0; j < CameraData.TagIDs.size(); j++)
		{
			if (markers[i].number == CameraData.TagIDs[j])
			{
				//gotcha!
				ArucoViewCameraLocal seen;
				seen.Marker = &markers[i];
				seen.IndexInCameraData = j;
				seen.CameraCornerPositions = CameraData.TagCorners[j];
				seen.AccumulatedTransform = AccumulatedTransform;
				vector<Point3d> &cornersLocal = markers[i].ObjectPointsNoOffset;
				Affine3d TransformToObject = AccumulatedTransform * markers[i].Pose;
				for (int k = 0; k < 4; k++)
				{
					seen.LocalMarkerCorners.push_back(TransformToObject * cornersLocal[k]);
				}
				MarkersSeen.push_back(seen);
				surface += contourArea(CameraData.TagCorners[j], false);
			}
			
		}
		
	}
	for (int i = 0; i < childs.size(); i++)
	{
		TrackedObject* child = childs[i];
		surface += child->GetSeenMarkers(CameraData, MarkersSeen, AccumulatedTransform * child->Location);
	}
	return surface;
}

float TrackedObject::ReprojectSeenMarkers(const std::vector<ArucoViewCameraLocal> &MarkersSeen, const Mat &rvec, const Mat &tvec, const CameraArucoData &CameraData)
{
	float ReprojectionError = 0;
	for (int i = 0; i < MarkersSeen.size(); i++)
	{
		vector<Point2d> cornersreproj;
		projectPoints(MarkersSeen[i].LocalMarkerCorners, rvec, tvec, CameraData.CameraMatrix, CameraData.DistanceCoefficients, cornersreproj);
		CameraData.SourceCamera->SetMarkerReprojection(MarkersSeen[i].IndexInCameraData, cornersreproj);
		for (int j = 0; j < 4; j++)
		{
			Point2f diff = MarkersSeen[i].CameraCornerPositions[j] - Point2f(cornersreproj[j]);
			ReprojectionError += sqrt(diff.ddot(diff));
		}
	}
	return ReprojectionError;
}

Affine3d TrackedObject::GetObjectTransform(const CameraArucoData& CameraData, float& Surface, float& ReprojectionError)
{
	vector<ArucoViewCameraLocal> SeenMarkers;
	Surface = GetSeenMarkers(CameraData, SeenMarkers, Affine3d::Identity());
	ReprojectionError = INFINITY;
	int nummarkersseen = SeenMarkers.size();

	if (nummarkersseen <= 0)
	{
		return Affine3d::Identity();
	}
	Affine3d localTransform;
	vector<Point3d> flatobj;
	vector<Point2f> flatimg;
	//vector<Point2d> flatreproj;
	flatobj.reserve(nummarkersseen * 4);
	flatimg.reserve(nummarkersseen * 4);
	Mat rvec = Mat::zeros(3, 1, CV_64F), tvec = Mat::zeros(3, 1, CV_64F);
	Affine3d objectToMarker;
	int flags = 0;
	if (nummarkersseen == 1)
	{
		flatobj = SeenMarkers[0].Marker->ObjectPointsNoOffset;
		SeenMarkers[0].LocalMarkerCorners = SeenMarkers[0].Marker->ObjectPointsNoOffset; //hack to have ReprojectSeenMarkers work wih a single marker too
		flatimg = SeenMarkers[0].CameraCornerPositions;
		objectToMarker = SeenMarkers[0].AccumulatedTransform * SeenMarkers[0].Marker->Pose;
		flags |= SOLVEPNP_IPPE_SQUARE;
	}
	else
	{
		for (int i = 0; i < nummarkersseen; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				flatobj.push_back(SeenMarkers[i].LocalMarkerCorners[j]);
				flatimg.push_back(SeenMarkers[i].CameraCornerPositions[j]);
			}
		}
		objectToMarker = Affine3d::Identity();
		flags |= CoplanarTags ? SOLVEPNP_IPPE : SOLVEPNP_SQPNP;
	}
	//Mat distCoeffs = Mat::zeros(4,1, CV_64F); //FIXME
	const Mat &distCoeffs = CameraData.DistanceCoefficients;
	solvePnP(flatobj, flatimg, CameraData.CameraMatrix, distCoeffs, rvec, tvec, false, flags);
	solvePnPRefineLM(flatobj, flatimg, CameraData.CameraMatrix, distCoeffs, rvec, tvec);
	ReprojectionError = ReprojectSeenMarkers(SeenMarkers, rvec, tvec, CameraData);
	
	ReprojectionError /= nummarkersseen;
	//cout << "Reprojection error : " << ReprojectionError << endl;
	Matx33d rotationMatrix; //Matrice de rotation Camera -> Tag
	Rodrigues(rvec, rotationMatrix);
	localTransform = Affine3d(rotationMatrix, tvec) * objectToMarker.inv();

	return localTransform;

	
}

vector<ObjectData> TrackedObject::ToObjectData(int BaseNumeral)
{
	cerr << "WARNING: Call to base TrackedObject::ToObjectData function. That function is uninplemented. Override it." <<endl;
	return {};
}

Affine3d GetTagTransform(float SideLength, std::vector<Point2f> Corners, Mat& CameraMatrix, Mat& DistanceCoefficients)
{
	Mat rvec, tvec;
	//Mat distCoeffs = Mat::zeros(4,1, CV_64F);
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

