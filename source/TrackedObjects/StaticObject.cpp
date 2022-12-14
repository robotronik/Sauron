#include "TrackedObjects/StaticObject.hpp"

#include <opencv2/calib3d.hpp>

#include "math3d.hpp"
#include "TrackedObjects/ObjectIdentity.hpp"
#include "data/CameraView.hpp"
#include "Cameras/Camera.hpp"

using namespace cv;
using namespace std;

StaticObject::StaticObject(bool InRelative, String InName)
{
	Unique = true;
	Relative = InRelative;
	Name = InName;
	const double yamp = 1-0.570, xamp = 1.5-0.575;
	double size = 0.1;
	vector<int> numbers = {20, 22, 21, 23};
	for (int i = 0; i < 4; i++)
	{
		Matx33d markerrot = MakeRotationFromZX(Vec3d(0,0,1), Vec3d(0,1,0));
		Vec3d pos = Vec3d(i%2 ? xamp : -xamp, i>=2 ? yamp : -yamp, 0);
		Affine3d markertransform = Affine3d(markerrot, pos);
		ArucoMarker marker(size, numbers[i], markertransform);
		if (marker.number >= 0)
		{
			markers.push_back(marker);
		}
	}
}

StaticObject::~StaticObject()
{
}

bool StaticObject::SetLocation(Affine3d InLocation)
{
	if (Relative)
	{
		Location = InLocation;
		return true;
	}
	
	Location = Affine3d::Identity();
	return false;
}

vector<ObjectData> StaticObject::ToObjectData(int BaseNumeral)
{
	ObjectData packet;
	packet.identity.type = Relative ? PacketType::ReferenceRelative : PacketType::ReferenceAbsolute;
	packet.identity.numeral = BaseNumeral;
	packet.location = Location;
	return {packet};
}

cv::Affine3d StaticObject::GetObjectTransform(const CameraArucoData& CameraData, float& Surface, float& ReprojectionError)
{
	return TrackedObject::GetObjectTransform(CameraData, Surface, ReprojectionError);
	vector<ArucoViewCameraLocal> SeenMarkers;
	Surface = GetSeenMarkers(CameraData, SeenMarkers, Affine3d::Identity());
	ReprojectionError = INFINITY;
	int nummarkersseen = SeenMarkers.size();

	if (nummarkersseen <= 0)
	{
		return Affine3d::Identity();
	}
	Affine3d localTransform;
	vector<Point3d> flatobj, flatobjtemp;
	vector<Point2f> flatimg, flatimgtemp;
	vector<Point2d> flatreproj;
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
	else if (nummarkersseen == 4)	
	{
		for (int i = 0; i < nummarkersseen; i++)
		{
			Point3d meanobj(0,0,0);
			Point2f meanimg(0,0);
			for (int j = 0; j < 4; j++)
			{
				meanobj += SeenMarkers[i].LocalMarkerCorners[j];
				meanimg += SeenMarkers[i].CameraCornerPositions[j];
			}
			flatobj.push_back(meanobj/4);
			flatimg.push_back(meanimg/4);
		}
		objectToMarker = Affine3d::Identity();
		flags |= SOLVEPNP_SQPNP;
	}
	else
	{
		Point3d mean(0,0,0);
		for (int i = 0; i < nummarkersseen; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				flatobjtemp.push_back(SeenMarkers[i].LocalMarkerCorners[j]);
				flatimgtemp.push_back(SeenMarkers[i].CameraCornerPositions[j]);
				mean += SeenMarkers[i].LocalMarkerCorners[j];
			}
		}
		mean /= nummarkersseen *4;
		if (nummarkersseen == 3)
		{
			mean = Point3d(0,0,0);
		}
		
		auto distancesq = [&mean, &flatobjtemp](int index)->float
		{
			Point3f delta = flatobjtemp[index] - mean;
			return delta.ddot(delta);
		};
		//keep only the 4 tags that are furthest apart from the mean
		int kept[4] = {0, 1, 2, 3};
		float distanceskept[4] = {distancesq(0), distancesq(1), distancesq(2), distancesq(3)};
		for (int i = 4; i < flatobjtemp.size(); i++)
		{
			float distloc = distancesq(i);
			int swapidx = -1;
			for (int kidx = 0; kidx < 4; kidx++)
			{
				if (swapidx == -1)
				{
					if (distloc > distanceskept[kidx])
					{
						swapidx = kidx;
					}
				}
				else
				{
					if (distanceskept[kidx] < distanceskept[swapidx])
					{
						swapidx = kidx;
					}
				}
			}
			if (swapidx != -1)
			{
				kept[swapidx] = i;
				distanceskept[swapidx] = distloc;
			}
		}
		for (int i = 0; i < 4; i++)
		{
			flatobj.push_back(flatobjtemp[kept[i]]);
			flatimg.push_back(flatimgtemp[kept[i]]);
		}
		
		
		objectToMarker = Affine3d::Identity();
		flags |= SOLVEPNP_SQPNP;
	}
	//Mat distCoeffs = Mat::zeros(4,1, CV_64F); //FIXME
	const Mat &distCoeffs = CameraData.DistanceCoefficients;
	solvePnP(flatobj, flatimg, CameraData.CameraMatrix, distCoeffs, rvec, tvec, false, flags);
	projectPoints(flatobj, rvec, tvec, CameraData.CameraMatrix, distCoeffs, flatreproj);
	ReprojectionError = ComputeReprojectionError(flatimg, flatreproj);
	CameraData.SourceCamera->SetMarkerReprojection(SeenMarkers[0].IndexInCameraData, flatreproj);
	if (nummarkersseen > 1)
	{
		vector<Point2d> fakeproj;
		for (int i = 0; i < 4; i++)
		{
			fakeproj.push_back(flatimg[i]);
		}
		
		CameraData.SourceCamera->SetMarkerReprojection(SeenMarkers[1].IndexInCameraData, fakeproj);
	}
	
	
	ReprojectionError /= nummarkersseen;
	//cout << "Reprojection error : " << ReprojectionError << endl;
	Matx33d rotationMatrix; //Matrice de rotation Camera -> Tag
	Rodrigues(rvec, rotationMatrix);
	localTransform = Affine3d(rotationMatrix, tvec) * objectToMarker.inv();

	return localTransform;
}