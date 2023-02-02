#include <iostream> // for standard I/O
#include <string>   // for strings
#include <sstream>  // string to number conversion
#include <math.h>

#include <thread>
#include <filesystem>

#include <opencv2/core.hpp>     // Basic OpenCV structures (Mat, Scalar)
#include <opencv2/core/ocl.hpp> //opencl
#include <opencv2/imgproc.hpp>

#include "data/CameraView.hpp"

#include "GlobalConf.hpp"
#include "Cameras/VideoCaptureCamera.hpp"
#include "Cameras/CameraManager.hpp"
#include "ObjectTracker.hpp"
#include "Calibrate.hpp"
#include "visualisation/BoardViz2D.hpp"
#include "visualisation/BoardViz3D.hpp"
#include "visualisation/BoardGL.hpp"
#include "Scenarios/CDFRExternal.hpp"
#include "Scenarios/CDFRInternal.hpp"

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#ifdef WITH_X11
#include <X11/Xlib.h>
#endif

using namespace std;
using namespace cv;


int main(int argc, char** argv )
{
	const string keys = 
		"{help h usage ? |      | print this message}"
		"{direct d       |      | show direct camera output}"
		"{build b        |      | print build information}"
		"{calibrate c    |      | start camera calibration wizard}"
		"{marker m       |      | print out markers}"
		"{cuda           |      | print cuda info}"
		"{board          |      | runs boardview test}"
		"{opengl ogl     |      | runs opengl test}"
		"{server s       |      | force server/client state}"
		;
	CommandLineParser parser(argc, argv, keys);

	if (parser.has("help"))
	{
		parser.printMessage();
		exit(EXIT_SUCCESS);
	}
	if (parser.has("build"))
	{
		cout << getBuildInformation() << endl;
		exit(EXIT_SUCCESS);
	}
	
	#ifdef WITH_CUDA
	cuda::setDevice(0);
	#endif
	ocl::setUseOpenCL(true);
	#ifdef WITH_X11
	XInitThreads();
	cout << "Detected screen resolution : " << GetScreenSize() << endl;
	#endif
	if (parser.has("cuda"))
	{
		#ifdef WITH_CUDA
		int cuda_devices_number = cuda::getCudaEnabledDeviceCount();
		cout << "CUDA Device(s) Number: "<< cuda_devices_number << endl;
		if (cuda_devices_number > 0)
		{
			cuda::DeviceInfo _deviceInfo;
			bool _is_device_compatible = _deviceInfo.isCompatible();
			cout << "CUDA Device(s) Compatible: " << _is_device_compatible << endl;
			cuda::printShortCudaDeviceInfo(cuda::getDevice());
		}
		#else
		cout << "CUDA is not enabled or detected on this device" << endl;
		#endif
		exit(EXIT_SUCCESS);
	}
	if (parser.has("board"))
	{
		TestBoardViz();
		exit(EXIT_SUCCESS);
	}
	if (parser.has("marker"))
	{
		Ptr<aruco::Dictionary> dictionary = GetArucoDict();
		std::filesystem::create_directory("../markers");
		//mkdir("../markers", 0777);
		for (int i = 0; i < 100; i++)
		{
			UMat markerImage;
			aruco::drawMarker(dictionary, i, 1024, markerImage, 1);
			char buffer[30];
			snprintf(buffer, sizeof(buffer)/sizeof(char), "../markers/marker%d.png", i);
			imwrite(buffer, markerImage);
		}
		Mat gigaimage;
		const int PixelsPerMM = 20;
		const int MarkerSizeMM = 50;
		const int MarkerSizeWhiteBorderMM = 55;
		const int SeparationMM = 1;
		const int MarkerSizePx = MarkerSizeMM * PixelsPerMM;
		const int MarkerSizeWhiteBorderPx = MarkerSizeWhiteBorderMM * PixelsPerMM;
		const int SeparationPixels = 10;
		Size GridMarkers(5,8);
		gigaimage = Mat(GridMarkers*MarkerSizeWhiteBorderPx + (GridMarkers+Size(1,1))*SeparationPixels, CV_8UC1, Scalar(0));
		cout << "Combined image should be scaled to " << (float)gigaimage.cols/PixelsPerMM << "x" << (float)gigaimage.rows/PixelsPerMM << "mm" << endl;
		int markeridx = 51;
		for (int j = 0; j < GridMarkers.height; j++)
		{
			for (int i = 0; i < GridMarkers.width; i++)
			{
				Point markerwhitestart(SeparationPixels + (SeparationPixels+MarkerSizeWhiteBorderPx)*i, SeparationPixels + (SeparationPixels+MarkerSizeWhiteBorderPx)*j);
				rectangle(gigaimage, markerwhitestart, markerwhitestart + Point(MarkerSizeWhiteBorderPx-1, MarkerSizeWhiteBorderPx-1), Scalar(255), FILLED);
				Point markerStart = markerwhitestart + Point(1,1) * ((MarkerSizeWhiteBorderPx-MarkerSizePx)/2);
				aruco::drawMarker(dictionary, markeridx, MarkerSizePx, gigaimage(Rect(markerStart, Size(MarkerSizePx, MarkerSizePx))), 1);
				markeridx++;
			}
			
		}
		imwrite("../markers/combined.png", gigaimage);
		exit(EXIT_SUCCESS);
	}
	
	vector<CameraSettings> CamSett = CameraManager::autoDetectCameras(GetCaptureMethod(), GetCaptureConfig().filter, "", false);

	if (CamSett.size() == 0)
	{
		cerr << "No cameras detected" << endl;
	}
	

	if (parser.has("calibrate"))
	{
		cout << "Starting calibration of camera index" << parser.get<int>("calibrate") <<endl;
		int camIndex = parser.get<int>("calibrate");
		if (0<= camIndex && camIndex < CamSett.size())
		{
			docalibration(CamSett[camIndex]);
			exit(EXIT_SUCCESS);
		}
		else
		{
			docalibration(CameraSettings());
			exit(EXIT_SUCCESS);
		}
		
		exit(EXIT_FAILURE);
		
	}

	bool direct = parser.has("direct");

	if(parser.has("server"))
	{
		GetWebsocketConfig().Server = parser.get<bool>("server");
		if (GetWebsocketConfig().Server)
		{
			cout << "Disregarding config, will be acting as server" <<endl;
		}
		else
		{
			cout << "Disregarding config, will be acting as client" <<endl;
		}
	}
	
	CDFRExternalMain(direct, true);
	
	// the camera will be deinitialized automatically in VideoCapture destructor
	return 0;
}