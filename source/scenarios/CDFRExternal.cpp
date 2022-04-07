#include "Scenarios/CDFRExternal.hpp"
#include "GlobalConf.hpp"
#include "data/FrameCounter.hpp"
#include "visualisation/BoardViz2D.hpp"
#include "visualisation/BoardViz3D.hpp"
#include "ObjectTracker.hpp"
#include "thirdparty/serialib.h"
#include "SerialSender.hpp"


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
				Cameras[CamIdx]->RescaleFrames(BufferIdx);
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

void CDFRExternalMain(bool direct)
{
    vector<Camera*> physicalCameras = autoDetectCameras(CameraStartType::GSTREAMER_CPU, "4", "Brio");
    Ptr<aruco::Dictionary> dictionary = GetArucoDict();

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
        namedWindow("Cameras", WINDOW_NORMAL);
		setWindowProperty("Cameras", WND_PROP_FULLSCREEN, WINDOW_FULLSCREEN);
		BoardViz2D::InitImages();
	}
	
	viz::Viz3d board3d("3D board");
	BoardViz3D::SetupTerrain(board3d);

	ObjectTracker tracker;
	tracker.SetArucoSize(center.number, center.sideLength);

    vector<String> SerialPorts = SerialSender::autoDetectTTYUSB();
    for (size_t i = 0; i < SerialPorts.size(); i++)
    {
        cout << "Serial port found at " << SerialPorts[i] << endl;
    }
    
	serialib* bridge = new serialib();
    if (SerialPorts.size() > 0)
    {
        int success = bridge->openDevice(SerialPorts[0].c_str(), 115200);
        cout << "Result opening serial bridge : " << success << endl;
		if (success != 1)
		{
			cout << "Failed to open the serial bridge, make sure your user is in the dialout group" <<endl;
			cout << "run this ->   sudo usermod -a -G dialout $USER    <- then restart your PC." << endl;
		}
		
    }
    
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
}