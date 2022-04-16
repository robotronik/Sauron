#pragma once

#include "GlobalConf.hpp"
#include "data/FrameCounter.hpp"
#include "Camera.hpp"
#include "visualisation/BoardViz3D.hpp"
#include "TrackedObjects/TrackerCube.hpp"
#include "ObjectTracker.hpp"
#include "thirdparty/serialib.h"
#include "SerialSender.hpp"

#include <fstream>
#include <iostream>
#include <opencv2/core.hpp>

using namespace cv;
using namespace std;


void BufferedPipeline(int BufferCaptureIdx, vector<Camera*> Cameras, 
    Ptr<aruco::Dictionary> dict, Ptr<aruco::DetectorParameters> params, ObjectTracker* registry);
