#include <iostream> // for standard I/O
#include <string>   // for strings
#include <sstream>  // string to number conversion
#include <math.h>

#include <filesystem>

#include <opencv2/core.hpp>     // Basic OpenCV structures (Mat, Scalar)
#include <opencv2/core/ocl.hpp>

#include "GlobalConf.hpp"
#include "data/OutputImage.hpp"
#include "Camera.hpp"
#include "TrackedObject.hpp"
#include "ObjectTracker.hpp"
#include "Calibrate.hpp"
#include "visualisation/BoardViz2D.hpp"
#include "visualisation/BoardViz3D.hpp"
#include "data/FrameCounter.hpp"
#include "SerialSender.hpp"
using namespace std;
using namespace cv;

vector<Camera*> physicalCameras;

void BufferedPipeline(int BufferCaptureIdx, vector<Camera*> Cameras, Ptr<aruco::Dictionary> dict, Ptr<aruco::DetectorParameters> params, ObjectTracker* registry)
{
	int numCams = Cameras.size();
	for (int i = 0; i < Cameras.size(); i++)
	{
		Cameras[i]->Grab(BufferCaptureIdx);
	}
	//Range range(0, numCams*Camera::FrameBufferSize);
	parallel_for_(Range(0, numCams * Camera::FrameBufferSize), [&](const Range& range)
	{
		for (int i = range.start; i < range.end; i++)
		{
			int BufferIdx = (BufferCaptureIdx + i/numCams) % Camera::FrameBufferSize;
			int CamIdx = i%numCams;
			switch (i/numCams)
			{
			case 0:
				// read
				
				
				break;
			
			case 1:
				//Detect aruco
				Cameras[CamIdx]->Read(BufferIdx);
				Cameras[CamIdx]->detectMarkers(BufferIdx, dict, params);
				Cameras[CamIdx]->SolveMarkers(BufferIdx, CamIdx, registry);
				break;

			default:
				cout << "Frame buffer too big or operation unimplemented" << endl;
				break;
			}
			
			
		}
		//cout << "Aruco stripe from " << range.start << " to " << range.end << endl;
	}, numCams*Camera::FrameBufferSize);
} 

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
	//ocl::setUseOpenCL(true);
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
    
	physicalCameras = autoDetectCameras(CameraStartType::GSTREAMER_CPU, "4", "Brio");

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
			docalibration(physicalCameras[camIndex]);
			exit(EXIT_SUCCESS);
		}
		exit(EXIT_FAILURE);
		
	}

	bool direct = parser.has("direct");
	
	if (direct)
	{
		namedWindow("Cameras", WINDOW_NORMAL);
		setWindowProperty("Cameras", WND_PROP_FULLSCREEN, WINDOW_FULLSCREEN);
	}
	
	

	StartCameras(physicalCameras);

	cout << "Start grabbing " << physicalCameras.size() << " physical" << endl
		<< "Press ESC to terminate" << endl;

	
	Ptr<aruco::DetectorParameters> parameters = GetArucoParams();
	FrameCounter fps;
	FrameCounter fpsRead, fpsDetect;
	FrameCounter fpsPipeline;
	int PipelineIdx = 0;
	BoardViz2D* board = new BoardViz2D(FVector2D<float>(3.0f, 2.0f), FVector2D<float>(1.5f, 1.0f));
	if (direct)
	{
		BoardViz2D::InitImages();
	}
	
	
	viz::Viz3d board3d("3D board");
	BoardViz3D::SetupTerrain(board3d);

	ObjectTracker tracker;
	tracker.SetArucoSize(center.number, center.sideLength);
	serialib* bridge = new serialib();
	SerialSender sender(bridge);
	TrackerCube* robot1 = new TrackerCube({51, 52, 54, 55}, 0.06, Point3d(0.0952, 0.0952, 0), "Robot1");
	TrackerCube* robot2 = new TrackerCube({57, 58, 59, 61}, 0.06, Point3d(0.0952, 0.0952, 0), "Robot2");
	tracker.RegisterTrackedObject(robot1);
	tracker.RegisterTrackedObject(robot2);

	sender.RegisterRobot(robot1);
	sender.RegisterRobot(robot2);

	int lastmarker = 0;
	for (;;)
	{
		fpsPipeline.GetDeltaTime();
		BufferedPipeline(PipelineIdx, physicalCameras, dictionary, parameters, &tracker);
		PipelineIdx = (PipelineIdx + 1) % Camera::FrameBufferSize;
		double TimePipeline = fpsPipeline.GetDeltaTime();

		//cout << "Pipeline took " << TimePipeline << "s to run" << endl;

		if (direct)
		{
			board->CreateBackground(Size(1500, 1000));
			board->OverlayImage(board->GetPalet(PaletCouleur::autre), FVector2D<float>(1), 0, FVector2D<float>(0.1));
		}
		
		vector<CameraView> views;
		vector<Affine3d> cameras;
		cameras.resize(physicalCameras.size());

		int viewsize = 0;
		for (int i = 0; i < physicalCameras.size(); i++)
		{
			viewsize += physicalCameras[i]->GetCameraViewsSize(PipelineIdx);
		}
		
		views.resize(viewsize);
		int viewsidx = 0;

		for (int i = 0; i < physicalCameras.size(); i++)
		{
			Camera* cam = physicalCameras[i];
			vector<CameraView> CameraViews;
			if (!cam->GetCameraViews(PipelineIdx, CameraViews))
			{
				continue;
			}
			Affine3d CamTransform = Affine3d::Identity();
			bool has42 = false;
			for (int mark = 0; mark < CameraViews.size(); mark++)
			{
				int markerid = CameraViews[mark].TagID;
				if (markerid == center.number)
				{
					CamTransform = center.Pose * CameraViews[mark].TagTransform.inv();
					cam->Location = CamTransform;
					has42 = true;
				}
				views[viewsidx++] = CameraViews[mark];
				
			}
			cameras[i] = cam->Location;
			BoardViz3D::ShowCamera(board3d, cam, PipelineIdx, cam->Location, has42 ? viz::Color::green() : viz::Color::red());
			//cout << "Camera" << i << " location : " << cam->Location.translation() << endl;
			
		}
		
		tracker.SolveLocations(cameras, views);
		tracker.DisplayObjects(&board3d);

		double deltaTime = fps.GetDeltaTime();

		if (direct)
		{
			vector<OutputImage*> OutputTargets;
			for (int i = 0; i < physicalCameras.size(); i++)
			{
				OutputTargets.push_back(physicalCameras[i]);
			}
			OutputTargets.push_back(board);
			
			UMat image = ConcatCameras(PipelineIdx, OutputTargets, OutputTargets.size());
			//board.GetOutputFrame(0, image, GetFrameSize());
			//cout << "Concat OK" <<endl;
			fps.AddFpsToImage(image, deltaTime);
			//printf("fps : %f\n", fps);
			imshow("Cameras", image);
		}
		
		
		viz::WText fpstext(to_string(1/deltaTime), Point2i(200,100));
		board3d.showWidget("fps", fpstext);
		board3d.spinOnce(1, true);

		sender.SendPacket();
		
		if (waitKey(1) == 27 || board3d.wasStopped())
		{
			break;
		}
	}
	// the camera will be deinitialized automatically in VideoCapture destructor
	return 0;
}