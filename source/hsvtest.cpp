#include "hsvtest.hpp"

#include "GlobalConf.hpp"
#include "data/FrameCounter.hpp"
#include "Camera.hpp"

String window_name = "HSVTest";
int V = 10;

void VChange(int newval, void* userdata)
{
	V = newval;
}

void hsvtest()
{
	vector<Camera*> physicalCameras = autoDetectCameras(CameraStartType::GSTREAMER_CPU, "Logi", "Brio");
	Ptr<aruco::Dictionary> dictionary = GetArucoDict();

	if (physicalCameras.size() < 1)
	{
		return;
	}
	StartCameras(physicalCameras);
	
	namedWindow(window_name);
    createTrackbar("Threshold V", window_name, NULL, 255, VChange);

	for (;;)
	{
		if (!physicalCameras[0]->Read(0))
		{
			cerr << "failed to read camera" << endl;
			continue;
		}
		UMat frameRGB, frameHSV, frameRange, frameGray;
		physicalCameras[0]->GetFrame(0, frameRGB);
		cvtColor(frameRGB, frameHSV, COLOR_BGR2HSV);
		inRange(frameHSV, Scalar(0,0,0), Scalar(255,255,V), frameRange);
		imshow(window_name, frameRange);
	}
	
}