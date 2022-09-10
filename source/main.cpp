#include <iostream> // for standard I/O
#include <string>   // for strings
#include <sstream>  // string to number conversion
#include <math.h>

#include <thread>
#include <filesystem>

#include <opencv2/core.hpp>     // Basic OpenCV structures (Mat, Scalar)
#include <opencv2/core/ocl.hpp> //opencl

#include "data/CameraView.hpp"

#include "GlobalConf.hpp"
#include "Cameras/VideoCaptureCamera.hpp"
#include "Cameras/ArgusCamera.hpp"
#include "ObjectTracker.hpp"
#include "Calibrate.hpp"
#include "visualisation/BoardViz2D.hpp"
#include "visualisation/BoardViz3D.hpp"
#include "Scenarios/CDFRExternal.hpp"
#include "Scenarios/CDFRInternal.hpp"

using namespace std;
using namespace cv;



int main(int argc, char** argv )
{
	//{
	//	ArgusCamera* CamTest = new ArgusCamera(CameraSettings());
	//	CamTest->StartFeed();
	//	//CamTest->Read(0);
	//	exit(EXIT_SUCCESS);
	//}
	//return ArgusEGLImage(argc, argv);
	const string keys = 
		"{help h usage ? |      | print this message}"
		"{direct d       |      | show direct camera output}"
		"{build b        |      | print build information}"
		"{calibrate c    |      | start camera calibration wizard}"
		"{marker m       |      | print out markers}"
		"{cuda           |      | print cuda info}"
		"{board          |      | runs boardview test}"
		"{viz3d          |      | runs viz3d test}"
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
	
	
	cuda::setDevice(0);
	ocl::setUseOpenCL(true);
	if (parser.has("cuda"))
	{
		int cuda_devices_number = cuda::getCudaEnabledDeviceCount();
		cout << "CUDA Device(s) Number: "<< cuda_devices_number << endl;
		if (cuda_devices_number > 0)
		{
			cuda::DeviceInfo _deviceInfo;
			bool _is_device_compatible = _deviceInfo.isCompatible();
			cout << "CUDA Device(s) Compatible: " << _is_device_compatible << endl;
			cuda::printShortCudaDeviceInfo(cuda::getDevice());
		}
		exit(EXIT_SUCCESS);
	}
	if (parser.has("board"))
	{
		TestBoardViz();
		exit(EXIT_SUCCESS);
	}
	if (parser.has("viz3d"))
	{
		TestViz3D();
		Tracker3DTest();
		exit(EXIT_SUCCESS);
	}
	Ptr<aruco::Dictionary> dictionary = GetArucoDict();
	if (parser.has("marker"))
	{
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
		exit(EXIT_SUCCESS);
	}
    
	vector<CameraSettings> CamSettings = Camera::autoDetectCameras(CameraStartType::GSTREAMER_CPU, "!HD User Facing", "", false);

	if (CamSettings.size() == 0)
	{
		cerr << "No cameras detected" << endl;
	}
	

	if (parser.has("calibrate"))
	{
		cout << "Starting calibration of camera index" << parser.get<int>("calibrate") <<endl;
		int camIndex = parser.get<int>("calibrate");
		if (0<= camIndex && camIndex < CamSettings.size())
		{
			docalibration(CamSettings[camIndex]);
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
	
	
	//hsvtest();
	CDFRExternalMain(direct, true);
	
	// the camera will be deinitialized automatically in VideoCapture destructor
	return 0;
}