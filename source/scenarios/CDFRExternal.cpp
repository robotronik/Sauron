#include "Scenarios/CDFRExternal.hpp"
#include "Scenarios/CDFRCommon.hpp"
#include "visualisation/BoardGL.hpp"
#include "data/ManualProfiler.hpp"
#include "data/senders/Encoders/MinimalEncoder.hpp"
#include "data/senders/Transport/TCPTransport.hpp"
#include "data/senders/Transport/UDPTransport.hpp"
#include <thread>

void CDFRExternalMain(bool direct, bool v3d)
{
	ManualProfiler prof("frames ");
	int ps = 0;
	prof.NameSection(ps++, "CameraMan Tick");
	prof.NameSection(ps++, "Camera Pipeline");
	prof.NameSection(ps++, "3D solve setup");
	prof.NameSection(ps++, "3D solve");
	prof.NameSection(ps++, "Visualizer");
	prof.NameSection(ps++, "Position packet");
	CameraManager CameraMan(GetCaptureMethod(), GetCaptureConfig().filter, false);

	auto& Detector = GetArucoDetector();

	vector<ArucoCamera*>& physicalCameras = CameraMan.Cameras;

	//display/debug section
	FrameCounter fps;
	if (direct)
	{
		namedWindow("Cameras", WINDOW_NORMAL);
		setWindowProperty("Cameras", WND_PROP_FULLSCREEN, WINDOW_FULLSCREEN);
		startWindowThread();
	}
	
	ObjectTracker tracker;

	StaticObject* boardobj = new StaticObject(false, "board");
	tracker.RegisterTrackedObject(boardobj); 
	TrackerCube* robot1 = new TrackerCube({51, 52, 54, 55}, 0.06, 0.0952, "Robot1");
	//TrackerCube* robot2 = new TrackerCube({57, 58, 59, 61}, 0.06, 0.0952, "Robot2");
	tracker.RegisterTrackedObject(robot1);
	//tracker.RegisterTrackedObject(robot2);
	
	BoardGL OpenGLBoard;
	//TrackerCube* testcube = new TrackerCube({51, 52, 53, 54, 55}, 0.05, 85.065/1000.0, "test");
	//OpenGLBoard.InspectObject(testcube);
	if (v3d)
	{
		OpenGLBoard.Start();
		OpenGLBoard.Tick({});
	}

	
	
	//track and untrack cameras dynamically
	CameraMan.PostCameraConnect = [&tracker](ArucoCamera* cam) -> bool
	{
		tracker.RegisterTrackedObject(cam);
		cout << "Registering new camera @" << cam << endl;
		return true;
	};
	CameraMan.OnDisconnect = [&tracker](ArucoCamera* cam) -> bool
	{
		tracker.UnregisterTrackedObject(cam);
		cout << "Unregistering camera @" << cam << endl;
		return true;
	};

	PositionDataSender sender;
	{
		WebsocketConfig wscfg = GetWebsocketConfig();
		sender.encoder = new MinimalEncoder(GetDefaultAllowMap());
		if (wscfg.TCP)
		{
			sender.transport = new TCPTransport(wscfg.Server, wscfg.IP, wscfg.Port, wscfg.Interface);
		}
		else
		{
			sender.transport = new UDPTransport(wscfg.Server, wscfg.IP, wscfg.Port, wscfg.Interface);
		}
		sender.StartReceiveThread();
	}

	int lastmarker = 0;
	for (;;)
	{

		double deltaTime = fps.GetDeltaTime();
		ps = 0;
		prof.EnterSection(ps++);
		CameraMan.Tick<VideoCaptureCamera>();
		prof.EnterSection(ps++);
		int64 GrabTick = getTickCount();
		BufferedPipeline(0, vector<ArucoCamera*>(physicalCameras.begin(), physicalCameras.end()), Detector, &tracker);
		prof.EnterSection(ps++);
		int NumCams = physicalCameras.size();
		vector<CameraArucoData> arucoDatas;
		vector<Affine3d> cameraLocations;
		vector<bool> CamerasWithPosition;
		cameraLocations.resize(NumCams);
		arucoDatas.resize(NumCams);
		CamerasWithPosition.resize(NumCams);
		int viewsidx = 0;

		for (int i = 0; i < NumCams; i++)
		{
			ArucoCamera* cam = physicalCameras[i];
			vector<CameraView> CameraViews;
			if (!cam->GetMarkerData(0, arucoDatas[i]))
			{
				continue;
			}

			bool hasposition = false;
			float surface, reprojectionError;
			Affine3d boardloc = boardobj->GetObjectTransform(arucoDatas[i], surface, reprojectionError);
			if (surface > 0)
			{

				cam->SetLocation(boardloc.inv(), GrabTick);
				hasposition = true;
			}
			arucoDatas[i].CameraTransform = cam->GetLocation();
			cameraLocations[i] = cam->GetLocation();
			CamerasWithPosition[i] = hasposition;
			
			//cout << "Camera " << i << " location : " << cameraLocations[i].translation() << " / Score: " << surface/(reprojectionError+0.1) << endl;
			
		}
		prof.EnterSection(ps++);
		tracker.SolveLocationsPerObject(arucoDatas, GrabTick);
		vector<ObjectData> ObjData = tracker.GetObjectDataVector();

		//Vec3d diff = robot1->GetLocation().translation() - robot2->GetLocation().translation(); 
		//cout << "Robot 1 location : " << robot1->GetLocation().translation() << endl;
		//cout << "Distance robot 1-2: " << sqrt(diff.ddot(diff)) << " m" <<endl;

		prof.EnterSection(ps++);
		
		if (v3d)
		{
			if(!OpenGLBoard.Tick(ObjData))
			{
				break;
			}
		}

		if (direct)
		{
			vector<OutputImage*> OutputTargets;
			OutputTargets.reserve(physicalCameras.size()+1);
			for (int i = 0; i < physicalCameras.size(); i++)
			{
				OutputTargets.push_back(physicalCameras[i]);
			}
			//OutputTargets.push_back(board);
			//board->DisplayData(ObjData);
			UMat image = ConcatCameras(0, OutputTargets, OutputTargets.size());
			//board.GetOutputFrame(0, image, GetFrameSize());
			//cout << "Concat OK" <<endl;
			fps.AddFpsToImage(image, deltaTime);
			//printf("fps : %f\n", fps);
			imshow("Cameras", image);
			if (pollKey() == '\e')
			{
				break;
			}
		}
		
		prof.EnterSection(ps++);
		if (GetWebsocketConfig().Server)
		{
			sender.SendPacket(GrabTick, ObjData);
			//this_thread::sleep_for(chrono::milliseconds(1000));
		}
		
		
		prof.EnterSection(-1);
		
		
		
		/*if (prof.ShouldPrint())
		{
			cout << fps.GetFPSString(deltaTime) << endl;
			prof.PrintIfShould();
		}	*/
		
	}
}