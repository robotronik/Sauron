#include <iostream> // for standard I/O
#include <string>   // for strings
#include <iomanip>  // for controlling float print precision
#include <sstream>  // string to number conversion
#include <math.h>
#include <opencv2/core.hpp>     // Basic OpenCV structures (Mat, Scalar)
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>  // OpenCV window I/O
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include <opencv2/core/cuda.hpp>
#include <opencv2/cudacodec.hpp>
#include <opencv2/cudaimgproc.hpp>

#include <opencv2/core/ocl.hpp>

#include "Camera.hpp"
#include "trackedobject.hpp"
#include "Calibrate.hpp"
#include "boardviz.hpp"
#include "FrameCounter.hpp"
using namespace std;
using namespace cv;

#define HAVE_CUDA

vector<Camera*> physicalCameras; //= {&cam0, &cam1};
vector<Camera*> virtualCameras;

void GrabReadCameras(vector<Camera*> Cameras, Ptr<aruco::Dictionary> dict, Ptr<aruco::DetectorParameters> params)
{
	for (int i = 0; i < Cameras.size(); i++)
	{
		Cameras[i]->Grab();
	}
	parallel_for_(Range(0, Cameras.size()), [&](const Range& range)
	{
		for (int i = range.start; i < range.end; i++)
		{
			if(Cameras[i]->Read())
			{
				//Cameras[i]->detectMarkers(dict, params);
			}
			//cout << "Read camera " << i << endl;
		}
	});
}

void DetectArucoCameras(vector<Camera*> Cameras, Ptr<aruco::Dictionary> dict, Ptr<aruco::DetectorParameters> params)
{
	parallel_for_(Range(0, Cameras.size()), [&](const Range& range)
	{
		for (int i = range.start; i < range.end; i++)
		{
			Cameras[i]->detectMarkers(dict, params);
		}
		//cout << "Aruco stripe from " << range.start << " to " << range.end << endl;
	});
}

UMat ConcatCameras(vector<Camera*> Cameras, int NumCams)
{
	Size screensize = Size(1920, 1080);
	UMat concatenated(screensize, CV_8UC3, Scalar(0,0,255));
	int rows = 1, columns = 1;
	while (rows * rows < NumCams)
	{
		rows++;
	}
	columns = rows;
	int winWidth = screensize.width/rows, winHeight = screensize.height/columns;
	parallel_for_(Range(0, Cameras.size()), [&](const Range& range)
	{
		for (int i = range.start; i < range.end; i++)
		{
			Rect roi(winWidth * (i%rows), winHeight * (i / rows), winWidth, winHeight);
			UMat frame; Cameras[i]->GetFrame(frame);
			if (frame.empty())
			{
				continue;
			}
			UMat region = concatenated(roi);
			Cameras[i]->GetOutputFrame(region, Size(winWidth, winHeight));
		}
	});
	return concatenated;
}

vector<Camera*> Cameras;

int main(int argc, char** argv )
{
	const string keys = 
		"{help h usage ? |      | print this message   }"
		"{build b        |      | print build information   }"
		"{calibrate c    |      | start camera calibration wizard   }"
		"{marker m       |      | print out markers               }"
		"{cuda           |      | print cuda info         }"
		"{board          |      | runs boardview test }"
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
			bool _isd_evice_compatible = _deviceInfo.isCompatible();
			cout << "CUDA Device(s) Compatible: " << _isd_evice_compatible << endl;
			cuda::printShortCudaDeviceInfo(cuda::getDevice());
		}
	}
	if (parser.has("board"))
	{
		TestBoardViz();
		exit(EXIT_SUCCESS);
	}
	Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_100);
	if (parser.has("marker"))
	{
		cout << "markers folder must exist before calling this function" << endl;
		for (int i = 0; i < 100; i++)
		{
			UMat markerImage;
			aruco::drawMarker(dictionary, i, 1024, markerImage, 1);
			char buffer[30];
			snprintf(buffer, sizeof(buffer), "../markers/marker%d.png", i);
			imwrite(buffer, markerImage);
		}
		exit(EXIT_SUCCESS);
	}
	
	
	
    
	physicalCameras = autoDetectCameras();

	if (physicalCameras.size() == 0)
	{
		cerr << "No cameras detected" << endl;
		exit(EXIT_FAILURE);
	}
	

	if (parser.has("calibrate"))
	{
		cout << "Starting calibration of camera index" << parser.get<int>("calibrate") <<endl;
		int camIndex = parser.get<int>("calibrate");
		if (0<= camIndex && camIndex < physicalCameras.size())
		{
			docalibration(physicalCameras[0]);
			exit(EXIT_SUCCESS);
		}
		exit(EXIT_FAILURE);
		
	}
	for (int i = 0; i < physicalCameras.size(); i++)
	{
		Cameras.push_back(physicalCameras[i]);
	}
	
	namedWindow("Cameras", WINDOW_NORMAL);
	setWindowProperty("Cameras", WND_PROP_FULLSCREEN, WINDOW_FULLSCREEN);

	StartCameras(physicalCameras);

	cout << "Start grabbing " << Cameras.size() << " cameras, " << physicalCameras.size() << " physical" << endl
		<< "Press any key to terminate" << endl;

	
	Ptr<aruco::DetectorParameters> parameters = aruco::DetectorParameters::create();
	parameters->cornerRefinementMethod = aruco::CORNER_REFINE_CONTOUR;
	FrameCounter fps;
	FrameCounter fpsRead, fpsDetect;
	for (;;)
	{
		fpsRead.GetDeltaTime();
		GrabReadCameras(physicalCameras, dictionary, parameters);
		double TimeRead = fpsRead.GetDeltaTime();
		fpsDetect.GetDeltaTime();
		DetectArucoCameras(physicalCameras, dictionary, parameters);
		double TimeDetect = fpsDetect.GetDeltaTime();

		cout << "Took " << TimeRead<< "s to read, " <<TimeDetect <<"s to detect" <<endl;

		for (int i = 0; i < Cameras.size(); i++)
		{
			Camera* cam = Cameras[i];
			if (cam->GetStatus() == CameraStatus::Arucoed)
			{
				for (int mark = 0; mark < cam->markerIDs.size(); mark++)
				{
					int markerid = cam->markerIDs[mark];
					switch (markerid)
					{
					case 42:
						{
							Mat rvec(3, 1, DataType<double>::type);
							Mat tvec(3, 1, DataType<double>::type);
							solvePnP(center.GetObjectPointsNoOffset(), cam->markerCorners[mark], cam->CameraMatrix, cam->distanceCoeffs, rvec, tvec, false, SOLVEPNP_IPPE_SQUARE);
							Point3d tvec2(tvec.at<double>(0,0), tvec.at<double>(1,0), tvec.at<double>(2,0));
							cout << sqrt(tvec2.dot(tvec2)) <<endl;
						}
						break;
					
					default:
						break;
					}
				}
				
			}
			
		}
		

		double deltaTime = fps.GetDeltaTime();
		UMat image = ConcatCameras(Cameras, Cameras.size());
		//cout << "Concat OK" <<endl;
		fps.AddFpsToImage(image, deltaTime);
		//printf("fps : %f\n", fps);
		imshow("Cameras", image);
		if (waitKey(5) >= 0)
			break;
		
	}
	// the camera will be deinitialized automatically in VideoCapture destructor
	
	return 0;
}