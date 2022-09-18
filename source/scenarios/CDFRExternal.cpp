#include "Scenarios/CDFRExternal.hpp"
#include "Scenarios/CDFRCommon.hpp"
#include "visualisation/BoardViz2D.hpp"
#include <thread>

void CDFRExternalMain(vector<CameraSettings> CameraSettings, bool direct, bool v3d)
{
    Ptr<aruco::Dictionary> dictionary = GetArucoDict();

    vector<VideoCaptureCamera*> physicalCameras = StartCameras<VideoCaptureCamera>(CameraSettings);

	cout << "Start grabbing " << physicalCameras.size() << " physical" << endl
		<< "Press ESC to terminate" << endl;

	
	Ptr<aruco::DetectorParameters> parameters = GetArucoParams();
	FrameCounter fps;
	BoardViz2D* board = nullptr;
	if (direct)
	{
		board = new BoardViz2D(FVector2D<float>(3.0f, 2.0f), FVector2D<float>(1.5f, 1.0f));
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
    
	SerialSender sender(true);

	StaticObject* boardobj = new StaticObject(false, "board");
	tracker.RegisterTrackedObject(boardobj); 
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


	if (v3d)
	{
		//board3d.spin();
	}
	
	int lastmarker = 0;
	for (;;)
	{
		BufferedPipeline(0, vector<ArucoCamera*>(physicalCameras.begin(), physicalCameras.end()), dictionary, parameters, &tracker);

		if (direct)
		{
			board->CreateBackground(Size(1500, 1000));
		}
		int NumCams = physicalCameras.size();
		vector<CameraView> views;
		vector<CameraArucoData> arucoDatas;
		vector<Affine3d> cameraLocations;
		vector<bool> CamerasWithPosition;
		cameraLocations.resize(NumCams);
		arucoDatas.resize(NumCams);
		CamerasWithPosition.resize(NumCams);
		int viewsize = 0;
		for (int i = 0; i < NumCams; i++)
		{
			viewsize += physicalCameras[i]->GetCameraViewsSize(0);
		}
		
		views.resize(viewsize);
		int viewsidx = 0;

		for (int i = 0; i < NumCams; i++)
		{
			VideoCaptureCamera* cam = physicalCameras[i];
			vector<CameraView> CameraViews;
			if (!cam->GetMarkerData(0, arucoDatas[i]))
			{
				continue;
			}
			
			if (!cam->GetCameraViews(0, CameraViews))
			{
				continue;
			}

			bool hasposition = false;
			float surface;
			Affine3d boardloc = boardobj->GetObjectTransform(arucoDatas[i], surface);
			if (surface > 0)
			{
				cam->Location = boardloc.inv();
				hasposition = true;
			}
			/*
			for (int mark = 0; mark < CameraViews.size(); mark++)
			{
				int markerid = CameraViews[mark].TagID;
				if (markerid == center.number)
				{
					CamTransform = center.Pose * CameraViews[mark].TagTransform.inv();
					cam->Location = CamTransform;
					hasposition = true;
				}
				views[viewsidx++] = CameraViews[mark];
				
			}*/
			cameraLocations[i] = cam->Location;
			CamerasWithPosition[i] = hasposition;
			
			//cout << "Camera" << i << " location : " << cam->Location.translation() << endl;
			
		}
		
		tracker.SolveLocationsPerObject(arucoDatas);

		double deltaTime = fps.GetDeltaTime();

		if (v3d)
		{
			if (board3d.wasStopped())
			{
				break;
			}
			
			tracker.DisplayObjects(&board3d);
			viz::WText fpstext(FrameCounter::GetFPSString(deltaTime), Point2i(100,150));
			board3d.showWidget("fps", fpstext);
			for (int i = 0; i < NumCams; i++)
			{
				BoardViz3D::ShowCamera(board3d, physicalCameras[i], 0, cameraLocations[i], CamerasWithPosition[i] ? viz::Color::green() : viz::Color::red());
			}
			board3d.spinOnce(1, true);
		}

		if (direct)
		{
			
			UMat image = ConcatCameras(0, OutputTargets, OutputTargets.size());
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
		
		
		
		if (waitKey(1) == 27)
		{
			break;
		}
	}
}