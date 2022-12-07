#pragma once

#include "TrackedObjects/TrackedObject.hpp"

class StaticObject : public TrackedObject
{
private:
	bool Relative; //Allow modifying the position of this object ?
public:
	StaticObject(bool InRelative, cv::String InName);
	~StaticObject();

	virtual bool SetLocation(cv::Affine3d InLocation) override;

	virtual std::vector<ObjectData> ToObjectData(int BaseNumeral) override;

	virtual cv::Affine3d GetObjectTransform(const CameraArucoData& CameraData, float& Surface, float& ReprojectionError) override;
};