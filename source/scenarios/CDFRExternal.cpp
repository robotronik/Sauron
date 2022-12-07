#include "Scenarios/CDFRExternal.hpp"
#include "Scenarios/CDFRCommon.hpp"
#include "visualisation/BoardViz2D.hpp"
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

	Ptr<aruco::Dictionary> dictionary = GetArucoDict();
	Ptr<aruco::DetectorParameters> parameters = GetArucoParams();

	vector<ArucoCamera*>& physicalCameras = CameraMan.Cameras;

	//display/debug section
	FrameCounter fps;
	BoardViz2D* board = nullptr;
	if (direct)
	{
		board = new BoardViz2D(FVector2D<float>(3.0f, 2.0f), FVector2D<float>(1.5f, 1.0f));
		namedWindow("Cameras", WINDOW_NORMAL);
		setWindowProperty("Cameras", WND_PROP_FULLSCREEN, WINDOW_FULLSCREEN);
		BoardViz2D::InitImages();
	}
	
	#ifdef WITH_VTK
	viz::Viz3d viz3dhandle("3D board");
	BoardViz3D board3d(&viz3dhandle);
	if (v3d)
	{
		BoardViz3D::SetupTerrain(viz3dhandle);
	}
	#endif

	BoardGL OpenGLBoard;
	OpenGLBoard.Start();
	OpenGLBoard.Tick({});

	ObjectTracker tracker;

	StaticObject* boardobj = new StaticObject(false, "board");
	tracker.RegisterTrackedObject(boardobj); 
	TrackerCube* robot1 = new TrackerCube({51, 52, 54, 55}, 0.06, Point3d(0.0952, 0.0952, 0), "Robot1");
	TrackerCube* robot2 = new TrackerCube({57, 58, 59, 61}, 0.06, Point3d(0.0952, 0.0952, 0), "Robot2");
	tracker.RegisterTrackedObject(robot1);
	tracker.RegisterTrackedObject(robot2);
	
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
		sender.encoder = new MinimalEncoder(UINT8_MAX-(uint8_t)PacketType::Camera);
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
		ps = 0;
		prof.EnterSection(ps++);
		CameraMan.Tick<VideoCaptureCamera>();
		prof.EnterSection(ps++);
		int64 GrabTick = getTickCount();
		BufferedPipeline(0, vector<ArucoCamera*>(physicalCameras.begin(), physicalCameras.end()), dictionary, parameters, &tracker);
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

				cam->SetLocation(boardloc.inv());
				hasposition = true;
			}
			arucoDatas[i].CameraTransform = cam->GetLocation();
			cameraLocations[i] = cam->GetLocation();
			CamerasWithPosition[i] = hasposition;
			
			//cout << "Camera " << i << " location : " << cameraLocations[i].translation() << " / Score: " << surface/(reprojectionError+0.1) << endl;
			
		}
		prof.EnterSection(ps++);
		tracker.SolveLocationsPerObject(arucoDatas);
		vector<ObjectData> ObjData = tracker.GetObjectDataVector();



		double deltaTime = fps.GetDeltaTime();
		prof.EnterSection(ps++);
		
		#ifdef WITH_VTK
		if (v3d)
		{
			if (viz3dhandle.wasStopped())
			{
				break;
			}
			
			board3d.DisplayData(ObjData);
			viz::WText fpstext(FrameCounter::GetFPSString(deltaTime), Point2i(100,150));
			viz3dhandle.showWidget("fps", fpstext);
			viz3dhandle.spinOnce(1, true);
		}
		#endif

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
		}

		if(!OpenGLBoard.Tick(ObjData))
		{
			break;
		}
		
		prof.EnterSection(ps++);
		sender.SendPacket(GrabTick, ObjData);
		
		
		
		if (waitKey(1) == '\e')
		{
			break;
		}
		prof.EnterSection(-1);
		if (prof.ShouldPrint())
		{
			cout << fps.GetFPSString(deltaTime) << endl;
			prof.PrintIfShould();
		}
		
		
	}
}