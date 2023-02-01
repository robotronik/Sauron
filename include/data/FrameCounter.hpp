#pragma once

#include <opencv2/core.hpp>
#include <string>

using std::string;

//Helper class to count frames per second
class FrameCounter
{
private:
	int64 dt;
	int64 startTime;
public:
	FrameCounter(/* args */);
	~FrameCounter();

	// Returns the time elapsed since last call of this function
	double GetDeltaTime();

	//Return time since creation of this object
	double GetAbsoluteTime();

	static string GetFPSString(double DeltaTime);

	//helper function to add a fps counter
	static void AddFpsToImage(cv::InputOutputArray img, double DeltaTime);
};
