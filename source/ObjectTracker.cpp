#include "ObjectTracker.hpp"
#include "data/CameraView.hpp"
#include "math3d.hpp"

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

void ObjectTracker::SolveLocationsPerObject(vector<CameraArucoData>& CameraData)
{
	
	
	/*parallel_for_(Range(0, objects.size()), [&](const Range& range)
	{*/
		Range range(0, objects.size());
		for(int ObjIdx = range.start; ObjIdx < range.end; ObjIdx++)
		{
			if (objects[ObjIdx]->markers.size() == 0)
			{
				continue;
			}
			float ScoreMax = 0;
			TrackedObject* object = objects[ObjIdx];
			vector<ResolvedLocation> locations;
			for (int CameraIdx = 0; CameraIdx < CameraData.size(); CameraIdx++)
			{
				float AreaThis, ReprojectionErrorThis;
				Affine3d transformProposed = CameraData[CameraIdx].CameraTransform * objects[ObjIdx]->GetObjectTransform(CameraData[CameraIdx], AreaThis, ReprojectionErrorThis);
				float ScoreThis = AreaThis/(ReprojectionErrorThis + 0.1);
				locations.emplace_back(ScoreThis, transformProposed, CameraData[CameraIdx].CameraTransform);
			}
			if (locations.size() == 0)
			{
				continue;
			}
			if (locations.size() == 1)
			{
				object->SetLocation(locations[0].AbsLoc);
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
			object->SetLocation(combinedloc);
			//cout << "Object " << ObjIdx << " is at location " << objects[ObjIdx]->GetLocation().translation() << " / score: " << ScoreMax << endl;
		}
	/*});*/
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

vector<ObjectData> ObjectTracker::GetObjectDataVector()
{
	vector<ObjectData> ObjectDatas;
	ObjectDatas.reserve(objects.size()*2);

	for (int i = 0; i < objects.size(); i++)
	{
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
		ArucoMap[object->markers[i].number] = index;
		ArucoSizes[object->markers[i].number] = object->markers[i].sideLength;
	}
	for (int i = 0; i < object->childs.size(); i++)
	{
		RegisterArucoRecursive(object->childs[i], index);
	}
}