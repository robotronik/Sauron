#pragma once

//File that provides most includes for both CDFR scenarios

#include <fstream>
#include <iostream>
#include <opencv2/core.hpp>


#include "GlobalConf.hpp"

#include "Cameras/VideoCaptureCamera.hpp"
#include "visualisation/BoardViz3D.hpp"
#include "TrackedObjects/TrackerCube.hpp"
#include "TrackedObjects/StaticObject.hpp"
#include "ObjectTracker.hpp"

#include "data/FrameCounter.hpp"
#include "data/CameraView.hpp"

#include "data/SerialPacket.hpp"
#include "SerialSender.hpp"


using namespace cv;
using namespace std;


void BufferedPipeline(int BufferCaptureIdx, vector<ArucoCamera*> Cameras, 
    Ptr<aruco::Dictionary> dict, Ptr<aruco::DetectorParameters> params, ObjectTracker* registry);
