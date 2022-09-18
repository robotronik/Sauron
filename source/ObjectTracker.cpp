#include "ObjectTracker.hpp"
#include "data/CameraView.hpp"

using namespace cv;
using namespace std;

ObjectTracker::ObjectTracker(/* args */)
{
	ArucoMap = new int[100];
	ArucoSizes = new float[100];

	for (int i = 0; i < 100; i++)
	{
		ArucoMap[i] = -1;
		ArucoSizes[i] = 0.05;
	}
	
}

ObjectTracker::~ObjectTracker()
{
	delete ArucoMap;
	delete ArucoSizes;
}

void ObjectTracker::RegisterTrackedObject(TrackedObject* object)
{
	int index = objects.size();
	objects.push_back(object);
	RegisterArucoRecursive(object, index);
}

void ObjectTracker::SolveLocationsPerObject(vector<CameraArucoData>& CameraData)
{
	parallel_for_(Range(0, objects.size()), [&](const Range& range)
	{
		for(int ObjIdx = range.start; ObjIdx < range.end; ObjIdx++)
		{
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
	});
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

void ObjectTracker::DisplayObjects(viz::Viz3d* visualizer)
{
	for (int i = 0; i < objects.size(); i++)
	{
		objects[i]->DisplayRecursive(visualizer, Affine3d::Identity(), "");
	}
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