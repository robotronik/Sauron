/*

	Ce fichier est le point d'entrée de l'application. Basée sur OpenCV pour le traitement d'images et de vidéos. Il s'agit d'une application de suivie d'objets en 3D avec une interface en ligne de commande. 

	Le programme utilise également les bibliothèque d'accélération matérielle CUDA et OpenCL pour accélérer le traitement d'image ainsi que la prise en charge du multi-threading pour accélérer le traitement des données.

*/


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
#include "visualisation/BoardGL.hpp"
#include "Scenarios/CDFRExternal.hpp"
#include "Scenarios/CDFRInternal.hpp"
#include "mapping.hpp"

#include "Overlord/Overlord.hpp"

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#ifdef WITH_X11
#include <X11/Xlib.h>
#endif

using namespace std;
using namespace cv;

// Fonction principale du programme
int main(int argc, char** argv )
{
	const string keys = 
		"{help h usage ? |      | print this message}"
		"{direct d       |      | show direct camera output}"
		"{build b        |      | print build information}"
		"{calibrate c    |      | start camera calibration wizard}"
		"{marker m       |      | print out markers}"
		"{cuda           |      | print cuda info}"
		"{opengl ogl     |      | runs opengl test}"
		"{server s       |      | force server/client state}"
		"{slam           |      | runs slam for object mapping, using saved images and calibration}"
		"{nodisplay | | start without a display}"
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
	
	bool nodisplay = parser.has("nodisplay");

	#ifdef WITH_CUDA
	cuda::setDevice(0);
	#endif
	ocl::setUseOpenCL(true);
	#ifdef WITH_X11
	if (nodisplay)
	{
		SetNoScreen();
	}
	else
	{
		XInitThreads();
		cout << "Detected screen resolution : " << GetScreenResolution() << endl;
		cout << "Detected screen size : " << GetScreenSize() << endl;
	}
	
	
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

	// L'option marker permet de générer des marqueurs
	if (parser.has("marker"))
	{
		auto& detector = GetArucoDetector();
		auto& dictionary = detector.getDictionary();
		std::filesystem::create_directory("../markers");
		//mkdir("../markers", 0777);
		for (int i = 0; i < 100; i++)
		{
			UMat markerImage;
			aruco::generateImageMarker(dictionary, i, 1024, markerImage, 1);
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
		const Size GridMarkers(8,5);
		gigaimage = Mat(GridMarkers*MarkerSizeWhiteBorderPx + (GridMarkers+Size(1,1))*SeparationPixels, CV_8UC1, Scalar(0));
		cout << "Combined image should be scaled to " << (float)gigaimage.cols/PixelsPerMM << "x" << (float)gigaimage.rows/PixelsPerMM << "mm" << endl;
		int markeridx = 51;
		const int endmarker = markeridx + GridMarkers.height*GridMarkers.width;
		for (int j = 0; j < GridMarkers.height; j++)
		{
			for (int i = 0; i < GridMarkers.width; i++)
			{
				if (markeridx >= 100)
				{
					break;
				}
				
				Point markerwhitestart(SeparationPixels + (SeparationPixels+MarkerSizeWhiteBorderPx)*i, SeparationPixels + (SeparationPixels+MarkerSizeWhiteBorderPx)*j);
				rectangle(gigaimage, markerwhitestart, markerwhitestart + Point(MarkerSizeWhiteBorderPx-1, MarkerSizeWhiteBorderPx-1), Scalar(255), FILLED);
				Point markerStart = markerwhitestart + Point(1,1) * ((MarkerSizeWhiteBorderPx-MarkerSizePx)/2);
				aruco::generateImageMarker(dictionary, markeridx, MarkerSizePx, gigaimage(Rect(markerStart, Size(MarkerSizePx, MarkerSizePx))), 1);
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
	
	// Permet de démarrer l'assistant de calibration des caméras
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
	bool opengl = !parser.has("nodisplay");

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
	
	if (parser.has("slam"))
	{
		SLAMSolve();
		exit(EXIT_SUCCESS);
	}


	// Selon les options fournies et configurations définies le programme principale est exécuté. Selon le type d'éxecution, le programme peut être un serveur, un client, un programme de test OpenGL ou un programme de test de caméra.
	
	switch (GetRunType())
	{
	case RunType::CameraExternal :
		cout << "Starting external camera program" <<endl;
		CDFRExternalMain(direct, opengl);
		break;
	case RunType::CameraInternal :
		cout << "Starting internal camera program" <<endl;
		CDFRInternalMain(direct, opengl);
		break;
	case RunType::Overlord : 
		cout << "Starting Overlord" << endl;
		{
			Overlord::Manager man;
			man.Thread(opengl, false);
		}
		break;
	case RunType::OverlordSim :
		cout << "Starting Overlord in simulation mode" << endl;
		{
			Overlord::Manager man;
			man.Thread(opengl, true);
		}
		break;
	default:
		cerr << "Run type unknown, nothing started" << endl;
		break;
	}
	
	
	
	
	// the camera will be deinitialized automatically in VideoCapture destructor
	return 0;
}