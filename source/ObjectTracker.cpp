#include "ObjectTracker.hpp"
#include "data/CameraView.hpp"

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
			float AreaMax = 0;
			for (int CameraIdx = 0; CameraIdx < CameraData.size(); CameraIdx++)
			{
				float AreaThis;
				Affine3d transformProposed = objects[ObjIdx]->GetObjectTransform(CameraData[CameraIdx], AreaThis);
				if (AreaThis > AreaMax)
				{
					objects[ObjIdx]->SetLocation(transformProposed);
					AreaMax = AreaThis;
				}
			}
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