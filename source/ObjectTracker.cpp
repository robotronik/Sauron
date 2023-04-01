#include "ObjectTracker.hpp"
#include "data/CameraView.hpp"
#include "math3d.hpp"
#include "TrackedObjects/StaticObject.hpp"

using namespace cv;
using namespace std;

ObjectTracker::ObjectTracker(/* args */)
{

	for (int i = 0; i < 100; i++)
	{
		ArucoMap[i] = -1;
		ArucoSizes[i] = 0.05;
	}
	
}

ObjectTracker::~ObjectTracker()
{
}

void ObjectTracker::RegisterTrackedObject(TrackedObject* object)
{
	int index = objects.size();
	objects.push_back(object);
	RegisterArucoRecursive(object, index);
}

void ObjectTracker::UnregisterTrackedObject(TrackedObject* object)
{
	auto objpos = find(objects.begin(), objects.end(), object);
	if (objpos != objects.end() && object->markers.size() == 0)
	{
		objects.erase(objpos);
	}
	
}

struct ResolvedLocation
{
	float score;
	Affine3d AbsLoc;
	Affine3d CameraLoc;

	ResolvedLocation(float InScore, Affine3d InObjLoc, Affine3d InCamLoc)
	:score(InScore), AbsLoc(InObjLoc), CameraLoc(InCamLoc)
	{}

	bool operator<(ResolvedLocation& other)
	{
		return score < other.score;
	}
};

void ObjectTracker::SolveLocationsPerObject(const vector<CameraArucoData>& CameraData, unsigned long tick)
{
	const int NumCameras = CameraData.size();
	const int NumObjects = objects.size();
	vector<CameraArucoData> BaseCameraData;
	BaseCameraData.resize(NumCameras);
	for (int i = 0; i < NumCameras; i++)
	{
		BaseCameraData[i].CameraMatrix = CameraData[i].CameraMatrix;
		BaseCameraData[i].CameraTransform = CameraData[i].CameraTransform;
		BaseCameraData[i].DistanceCoefficients = CameraData[i].DistanceCoefficients;
		BaseCameraData[i].SourceCamera = CameraData[i].SourceCamera;
	}
	
	vector<vector<CameraArucoData>> DataPerObject;
	DataPerObject.resize(NumObjects, BaseCameraData);
	
	for (int i = 0; i < NumCameras; i++)
	{
		const CameraArucoData& data = CameraData[i];
		for (int j = 0; j < data.TagIDs.size(); j++)
		{
			int tagidx = data.TagIDs[j];
			int objectidx = ArucoMap[tagidx];
			if (objectidx < 0 || objectidx >= NumObjects)
			{
				continue;
			}
			DataPerObject[objectidx][i].TagCorners.push_back(data.TagCorners[j]);
			DataPerObject[objectidx][i].TagIDs.push_back(tagidx);
		}
	}
	/*parallel_for_(Range(0, objects.size()), [&](const Range& range)
	{*/
		Range range(0, objects.size());
		for(int ObjIdx = range.start; ObjIdx < range.end; ObjIdx++)
		{
			TrackedObject* object = objects[ObjIdx];
			if (object->markers.size() == 0)
			{
				continue;
			}
			StaticObject* objstat = dynamic_cast<StaticObject*>(object);
			if (objstat != nullptr)
			{
				if (!objstat->IsRelative()) //Do not solve for static non-relative objects
				{
					continue;
				}
			}
			
			vector<ResolvedLocation> locations;
			for (int CameraIdx = 0; CameraIdx < CameraData.size(); CameraIdx++)
			{
				const CameraArucoData& ThisCameraData = DataPerObject[ObjIdx][CameraIdx];
				if (ThisCameraData.TagIDs.size() == 0) //Not seen
				{
					continue;
				}
				float AreaThis, ReprojectionErrorThis;
				Affine3d transformProposed = ThisCameraData.CameraTransform * objects[ObjIdx]->GetObjectTransform(ThisCameraData, AreaThis, ReprojectionErrorThis);
				float ScoreThis = AreaThis/(ReprojectionErrorThis + 0.1);
				if (ScoreThis < 1 || ReprojectionErrorThis == INFINITY) //Bad solve or not seen
				{
					continue;
				}
				locations.emplace_back(ScoreThis, transformProposed, ThisCameraData.CameraTransform);
			}
			if (locations.size() == 0)
			{
				continue;
			}
			if (locations.size() == 1)
			{
				object->SetLocation(locations[0].AbsLoc, tick);
				//cout << "Object " << object->Name << " is at location " << objects[ObjIdx]->GetLocation().translation() << " / score: " << locations[0].score << ", seen by 1 camera" << endl;
				continue;
			}
			std::sort(locations.begin(), locations.end());
			ResolvedLocation &best = locations[locations.size()-1];
			ResolvedLocation &secondbest = locations[locations.size()-2];
			Vec3d l1p = best.AbsLoc.translation();
			Vec3d l2p = secondbest.AbsLoc.translation();
			Vec3d l1d = NormaliseVector(best.CameraLoc.translation() - l1p);
			Vec3d l2d = NormaliseVector(secondbest.CameraLoc.translation() - l2p);
			Vec3d l1i, l2i;
			ClosestPointsOnTwoLine(l1p, l1d, l2p, l2d, l1i, l2i);
			Vec3d locfinal = (l1i*best.score+l2i*secondbest.score)/(best.score + secondbest.score);
			Affine3d combinedloc = best.AbsLoc;
			combinedloc.translation(locfinal);
			object->SetLocation(combinedloc, tick);
			//cout << "Object " << object->Name << " is at location " << objects[ObjIdx]->GetLocation().translation() << " / score: " << best.score+secondbest.score << ", seen by " << locations.size() << " cameras" << endl;
		}
	//});
}

void ObjectTracker::SolveLocationsTagByTag(vector<Affine3d>& Cameras, vector<CameraView>& Tags)
{
	vector<vector<CameraView>> tagsForObject;
	tagsForObject.resize(objects.size());
	//assign each view of a tag to an object
	for (int i = 0; i < Tags.size(); i++)
	{
		int ObjectIdx = ArucoMap[Tags[i].TagID];
		if (ObjectIdx <0) //no object
		{
			continue;
		}
		tagsForObject[ObjectIdx].push_back(Tags[i]);
	}
	for (int ObjIdx = 0; ObjIdx < tagsForObject.size(); ObjIdx++)
	{
		vector<CameraView> arucos;
		if (tagsForObject[ObjIdx].size() == 0)
		{
			continue;
		}
		
		arucos.resize(tagsForObject[ObjIdx].size());
		for (int i = 0; i < arucos.size(); i++)
		{
			CameraView& v = tagsForObject[ObjIdx][i];
			arucos[i] = v;
		}
		objects[ObjIdx]->ResolveLocation(Cameras, arucos);
	}
}

vector<ObjectData> ObjectTracker::GetObjectDataVector(unsigned long Tick)
{
	vector<ObjectData> ObjectDatas;
	ObjectDatas.reserve(objects.size()*2);

	for (int i = 0; i < objects.size(); i++)
	{
		if (Tick != 0 && !objects[i]->ShouldBeDisplayed(Tick)) //not seen, do not display
		{
			continue;
		}
		
		vector<ObjectData> lp = objects[i]->ToObjectData(i);
		for (int j = 0; j < lp.size(); j++)
		{
			ObjectDatas.push_back(lp[j]);
		}
	}
	return ObjectDatas;
}

void ObjectTracker::SetArucoSize(int number, float SideLength)
{
	ArucoSizes[number] = SideLength;
}

float ObjectTracker::GetArucoSize(int number)
{
	return ArucoSizes[number];
}

void ObjectTracker::RegisterArucoRecursive(TrackedObject* object, int index)
{
	for (int i = 0; i < object->markers.size(); i++)
	{
		const ArucoMarker& marker = object->markers[i];
		int MarkerID = marker.number;
		if (ArucoMap[MarkerID] != -1)
		{
			cerr << "WARNING Overwriting Marker data/owner for marker index " << MarkerID << " with object " << object->Name << endl;
		}
		ArucoMap[MarkerID] = index;
		ArucoSizes[MarkerID] = marker.sideLength;
	}
	for (int i = 0; i < object->childs.size(); i++)
	{
		RegisterArucoRecursive(object->childs[i], index);
	}
}