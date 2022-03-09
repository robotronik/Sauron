#include "ObjectTracker.hpp"

ObjectTracker::ObjectTracker(/* args */)
{
	ArucoMap = new int[100];

	for (int i = 0; i < 100; i++)
	{
		ArucoMap[i] = -1;
	}
	
}

ObjectTracker::~ObjectTracker()
{
	delete ArucoMap;
}

void ObjectTracker::RegisterTrackedObject(TrackedObject* object)
{
	int index = objects.size();
	objects.push_back(object);
	RegisterArucoRecursive(object, index);
}

void ObjectTracker::SolveLocations(vector<Affine3d>& Cameras, vector<CameraView>& Tags)
{
	vector<vector<CameraView>> tagsForObject;
	tagsForObject.resize(objects.size());
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


void ObjectTracker::RegisterArucoRecursive(TrackedObject* object, int index)
{
	for (int i = 0; i < object->markers.size(); i++)
	{
		ArucoMap[object->markers[i].number] = index;
	}
	for (int i = 0; i < object->childs.size(); i++)
	{
		RegisterArucoRecursive(object->childs[i], index);
	}
}