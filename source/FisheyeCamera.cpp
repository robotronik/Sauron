#include "FisheyeCamera.hpp"


#include <iostream> // for standard I/O
#include <iomanip>  // for controlling float print precision
#include <sstream>  // string to number conversion
#include <opencv2/imgproc.hpp>
#include <opencv2/cudacodec.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/calib3d.hpp>
#include "thirdparty/list-devices.hpp"
//#include "thirdparty/vidcap.h"
#include "data/Calibfile.hpp"
#include "TrackedObjects/TrackedObject.hpp" //CameraView
#include "ObjectTracker.hpp"
#include "data/FrameCounter.hpp"
#include "GlobalConf.hpp"
#include "data/FVector2D.hpp"

using namespace std;


void FisheyeCamera::Undistort(int BufferIdx)
{
	CameraSettings setcopy = GetCameraSettings();
	if (!HasUndistortionMaps)
	{
		Mat map1, map2;
		fisheye::initUndistortRectifyMap(setcopy.CameraMatrix, setcopy.distanceCoeffs, Mat::eye(3,3, CV_64F), 
		setcopy.CameraMatrix, setcopy.Resolution, CV_32FC1, map1, map2);
		UndistMap1.upload(map1);
		UndistMap2.upload(map2);
		HasUndistortionMaps = true;
	}
	BufferedFrame& buff = FrameBuffer[BufferIdx];
	
	if (!buff.FrameRaw.MakeGPUAvailable())
	{
		return;
	}
	
	buff.FrameUndistorted.GPUFrame = cuda::createContinuous(buff.FrameRaw.GPUFrame.size(), buff.FrameRaw.GPUFrame.type());
	//cout << "CV_32F=" << CV_32F << " 1=" << UndistMap1.type() << " 2=" << UndistMap2.type() << endl;
	cuda::remap(buff.FrameRaw.GPUFrame, buff.FrameUndistorted.GPUFrame, UndistMap1, UndistMap2, INTER_LINEAR);
	buff.FrameUndistorted.HasGPU = true;
	buff.FrameUndistorted.HasCPU = false;
}

//Create lower-resolution copies of the frame to be used in aruco detection
void FisheyeCamera::RescaleFrames(int BufferIdx)
{
	BufferedFrame& buff = FrameBuffer[BufferIdx];
	if (!buff.FrameUndistorted.MakeGPUAvailable())
	{
		return;
	}

	vector<Size> rescales = GetArucoReductions();
	buff.rescaledFrames.resize(rescales.size());

	for (int i = 0; i < rescales.size(); i++)
	{
		cuda::GpuMat rescaled;
		cuda::resize(buff.FrameUndistorted.GPUFrame, rescaled, rescales[i], 0, 0, INTER_AREA);
		cuda::cvtColor(rescaled, buff.rescaledFrames[i].GPUFrame, COLOR_BGR2GRAY);
		buff.rescaledFrames[i].HasGPU = true;
		buff.rescaledFrames[i].HasCPU = false;
	}
}

void FisheyeCamera::Calibrate(vector<vector<Point3f>> objectPoints,
	vector<vector<Point2f>> imagePoints, Size imageSize,
	Mat& cameraMatrix, Mat& distCoeffs,
	OutputArrayOfArrays rvecs, OutputArrayOfArrays tvecs) 
{
	cameraMatrix = Mat::eye(3, 3, CV_64F);
	distCoeffs = Mat::zeros(Size(4,1), CV_64F);
	fisheye::calibrate(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, 
		rvecs, tvecs, fisheye::CALIB_RECOMPUTE_EXTRINSIC | fisheye::CALIB_CHECK_COND | fisheye::CALIB_FIX_SKEW);
}