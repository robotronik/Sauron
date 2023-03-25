#pragma once

#include "TrackedObjects/TrackedObject.hpp"

//An object that doesn't move and has a location fixed on the terrain, such as the table or the basket
class StaticObject : public TrackedObject
{
private:
	bool Relative; //Allow modifying the position of this object ?
public:
	StaticObject(bool InRelative, cv::String InName);
	~StaticObject();

	bool IsRelative()
	{
		return Relative;
	}

	virtual bool SetLocation(cv::Affine3d InLocation, double dt) override;

	virtual std::vector<ObjectData> ToObjectData(int BaseNumeral) override;

	//virtual cv::Affine3d GetObjectTransform(const CameraArucoData& CameraData, float& Surface, float& ReprojectionError) override;
};