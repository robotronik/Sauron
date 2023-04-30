#pragma once

#include "TrackedObjects/TrackedObject.hpp"

//A cube with 4 tags, one on each side
class TopTracker : public TrackedObject
{
private:

public:
	TopTracker(int MarkerIdx, double MarkerSize, std::string InName);
	~TopTracker();

	virtual std::vector<ObjectData> ToObjectData(int BaseNumeral) override;
};