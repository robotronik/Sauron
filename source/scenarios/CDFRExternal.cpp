#include "Scenarios/CDFRExternal.hpp"
#include "Scenarios/CDFRCommon.hpp"
#include "visualisation/BoardViz2D.hpp"

void CDFRExternalMain(bool direct, bool v3d)
{
    vector<CameraSettings> CameraSettings = Camera::autoDetectCameras(CameraStartType::GSTREAMER_CPU, "", "Brio");
    Ptr<aruco::Dictionary> dictionary = GetArucoDict();

    vector<VideoCaptureCamera*> physicalCameras = StartCameras<VideoCaptureCamera>(CameraSettings);

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
	
	viz::Viz3d board3d;
	if (v3d)
	{
		board3d = viz::Viz3d("3D board");
		BoardViz3D::SetupTerrain(board3d);
	}

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
        int success = bridge->openDevice(SerialPorts[0].c_str(), SerialTransmission::BaudRate);
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

	sender.RegisterTrackedObject(robot1);
	sender.RegisterTrackedObject(robot2);

	//ofstream printfile;
	//printfile.open("logpos.csv", ios::out);

	//sender.PrintCSVHeader(printfile);

	vector<OutputImage*> OutputTargets;
	for (int i = 0; i < physicalCameras.size(); i++)
	{
		OutputTargets.push_back(physicalCameras[i]);
	}
	//OutputTargets.push_back(board);
	
	int lastmarker = 0;
	for (;;)
	{
		fpsPipeline.GetDeltaTime();
		BufferedPipeline(PipelineIdx, vector<ArucoCamera*>(physicalCameras.begin(), physicalCameras.end()), dictionary, parameters, &tracker);
		PipelineIdx = (PipelineIdx + 1) % 2;
		double TimePipeline = fpsPipeline.GetDeltaTime();

		//cout << "Pipeline took " << TimePipeline << "s to run" << endl;

		if (direct)
		{
			board->CreateBackground(Size(1500, 1000));
		}
		
		vector<CameraView> views;
		vector<CameraArucoData> arucoDatas;
		vector<Affine3d> cameraLocations;
		cameraLocations.resize(physicalCameras.size());
		arucoDatas.resize(physicalCameras.size());
		int viewsize = 0;
		for (int i = 0; i < physicalCameras.size(); i++)
		{
			viewsize += physicalCameras[i]->GetCameraViewsSize(PipelineIdx);
		}
		
		views.resize(viewsize);
		int viewsidx = 0;

		for (int i = 0; i < physicalCameras.size(); i++)
		{
			VideoCaptureCamera* cam = physicalCameras[i];
			vector<CameraView> CameraViews;
			if (!cam->GetMarkerData(PipelineIdx, arucoDatas[i]))
			{
				continue;
			}
			
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
			cameraLocations[i] = cam->Location;
			if (v3d)
			{
				BoardViz3D::ShowCamera(board3d, cam, PipelineIdx, cam->Location, has42 ? viz::Color::green() : viz::Color::red());
			}
			
			//cout << "Camera" << i << " location : " << cam->Location.translation() << endl;
			
		}
		
		tracker.SolveLocationsTagByTag(cameraLocations, views);

		double deltaTime = fps.GetDeltaTime();

		if (v3d)
		{
			tracker.DisplayObjects(&board3d);
			viz::WText fpstext(to_string(1/deltaTime), Point2i(200,100));
			board3d.showWidget("fps", fpstext);
			board3d.spinOnce(1, true);
		}

		if (direct)
		{
			
			UMat image = ConcatCameras(PipelineIdx, OutputTargets, OutputTargets.size());
			//board.GetOutputFrame(0, image, GetFrameSize());
			//cout << "Concat OK" <<endl;
			fps.AddFpsToImage(image, deltaTime);
			//printf("fps : %f\n", fps);
			imshow("Cameras", image);
		}
		
		
		
		//sender.PrintCSV(printfile);
		/*sender.SendPacket();
		if (bridge->available() > 0)
		{
			int out = bridge->readString(rcvbuff, '\n', rcvbuffsize, 1);
			if (out > 0)
			{
				printf("Received from serial : %*s", out, rcvbuff);
			}
		}*/
		
		
		
		if (waitKey(1) == 27 || (v3d && board3d.wasStopped()))
		{
			break;
		}
	}
}