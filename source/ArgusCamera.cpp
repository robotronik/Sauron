#include "ArgusCamera.hpp"


#include <iostream> // for standard I/O
#include <iomanip>  // for controlling float print precision
#include <sstream>  // string to number conversion
#include <opencv2/imgproc.hpp>
#include <opencv2/cudacodec.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/calib3d.hpp>
#include "thirdparty/list-devices.hpp"
#include "data/CameraView.hpp"

using namespace std;


bool ArgusCamera::Grab(int BufferIdx)
{
	return false;
}

bool ArgusCamera::Read(int BufferIdx)
{
	return false;
}