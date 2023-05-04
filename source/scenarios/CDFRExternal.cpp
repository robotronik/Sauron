#include "Scenarios/CDFRExternal.hpp"
#include "Scenarios/CDFRCommon.hpp"
#include "visualisation/BoardGL.hpp"
#include "data/ManualProfiler.hpp"
#include "data/senders/Encoders/MinimalEncoder.hpp"
#include "data/senders/Encoders/TextEncoder.hpp"
#include "data/senders/Transport/TCPTransport.hpp"
#include "data/senders/Transport/UDPTransport.hpp"
#include "data/senders/Transport/FileTransport.hpp"
#include <thread>
#include <memory>

CDFRTeam GetTeamFromCameras(vector<ArucoCamera*> Cameras)
{
	if (Cameras.size() == 0)
	{
		return CDFRTeam::Unknown;
	}
	int blues = 0, greens = 0;
	const static double ydist = 1.1;
	const static double xdist = 1.45;
	const static map<CDFRTeam, vector<Vec2d>> CameraPos = 
	{
		{CDFRTeam::Blue, {{0, ydist}, {-xdist, -ydist}, {xdist, -ydist}, {-1.622, -0.1}}},
		{CDFRTeam::Green, {{0, -ydist}, {-xdist, ydist}, {xdist, ydist}, {-1.622, 0.1}}}
	};
	map<CDFRTeam, int> TeamScores;
	for (ArucoCamera* cam : Cameras)
	{
		auto campos = cam->GetLocation().translation();
		Vec2d pos2d(campos[0], campos[1]);
		CDFRTeam bestTeam = CDFRTeam::Unknown;
		double bestdist = 0.2; //tolerance of 20cm
		for (const auto [team, positions] : CameraPos)
		{
			for (const auto position : positions)
			{
				Vec2d delta = position-pos2d;
				double dist = sqrt(delta.ddot(delta));
				if (dist < bestdist)
				{
					bestTeam = team;
					bestdist = dist;
				}
			}
			
		}
		auto position = TeamScores.find(bestTeam);
		if (position == TeamScores.end())
		{
			TeamScores[bestTeam] = 1;
		}
		else
		{
			TeamScores[bestTeam] += 1;
		}
	}
	CDFRTeam bestTeam = CDFRTeam::Unknown;
	int mostCount = 0;
	for (const auto [team, count] : TeamScores)
	{
		if (count > mostCount)
		{
			bestTeam = team;
			mostCount = count;
		}
	}
	return bestTeam;
}

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
	
	ObjectTracker bluetracker, greentracker;

	unique_ptr<StaticObject> boardobj = make_unique<StaticObject>(false, "board");
	bluetracker.RegisterTrackedObject(boardobj.get()); 
	greentracker.RegisterTrackedObject(boardobj.get());
	//TrackerCube* robot1 = new TrackerCube({51, 52, 54, 55}, 0.06, 0.0952, "Robot1");
	//TrackerCube* robot2 = new TrackerCube({57, 58, 59, 61}, 0.06, 0.0952, "Robot2");
	//bluetracker.RegisterTrackedObject(robot1);
	//bluetracker.RegisterTrackedObject(robot2);
	
	BoardGL OpenGLBoard;
	unique_ptr<TrackerCube> blue1 = make_unique<TrackerCube>(vector<int>({51, 52, 53, 54, 55}), 0.05, 85.065/1000.0, "blue1");
	unique_ptr<TrackerCube> blue2 = make_unique<TrackerCube>(vector<int>({56, 57, 58, 59, 60}), 0.05, 85.065/1000.0, "blue2");
	unique_ptr<TrackerCube> green1 = make_unique<TrackerCube>(vector<int>({71, 72, 73, 74, 75}), 0.05, 85.065/1000.0, "green1");
	unique_ptr<TrackerCube> green2 = make_unique<TrackerCube>(vector<int>({76, 77, 78, 79, 80}), 0.05, 85.065/1000.0, "green2");
	bluetracker.RegisterTrackedObject(blue1.get());
	bluetracker.RegisterTrackedObject(blue2.get());

	greentracker.RegisterTrackedObject(green1.get());
	greentracker.RegisterTrackedObject(green2.get());

	vector<unique_ptr<TopTracker>> TopTrackers;
	for (int i = 1; i < 11; i++)
	{
		TopTracker* tt = new TopTracker(i, 0.07, "top tracker " + std::to_string(i));
		TopTrackers.emplace_back(tt);
		bluetracker.RegisterTrackedObject(tt);
		greentracker.RegisterTrackedObject(tt);
	}
	
	
	//OpenGLBoard.InspectObject(blue1);
	if (v3d)
	{
		OpenGLBoard.Start();
		OpenGLBoard.Tick({});
	}

	
	
	//track and untrack cameras dynamically
	CameraMan.PostCameraConnect = [&bluetracker, &greentracker](ArucoCamera* cam) -> bool
	{
		bluetracker.RegisterTrackedObject(cam);
		greentracker.RegisterTrackedObject(cam);
		cout << "Registering new camera @" << cam << endl;
		return true;
	};
	CameraMan.OnDisconnect = [&bluetracker, &greentracker](ArucoCamera* cam) -> bool
	{
		bluetracker.UnregisterTrackedObject(cam);
		greentracker.UnregisterTrackedObject(cam);
		cout << "Unregistering camera @" << cam << endl;
		return true;
	};

	PositionDataSender sender;
	PositionDataSender logger;
	{
		WebsocketConfig wscfg = GetWebsocketConfig();
		sender.encoder = new MinimalEncoder(GetDefaultAllowMap());
		logger.encoder = new TextEncoder(GetDefaultAllowMap());
		if (wscfg.TCP)
		{
			sender.transport = new TCPTransport(wscfg.Server, wscfg.IP, wscfg.Port, wscfg.Interface);
		}
		else
		{
			sender.transport = new UDPTransport(wscfg.Server, wscfg.IP, wscfg.Port, wscfg.Interface);
		}
		logger.transport = new FileTransport("PositionLog.txt");
		sender.StartReceiveThread();
	}

	int lastmarker = 0;
	CDFRTeam LastTeam = CDFRTeam::Unknown;
	for (;;)
	{

		double deltaTime = fps.GetDeltaTime();
		ps = 0;
		prof.EnterSection(ps++);
		CameraMan.Tick<VideoCaptureCamera>();
		CDFRTeam Team = GetTeamFromCameras(CameraMan.Cameras);
		if (Team != LastTeam)
		{
			const string teamname = TeamNames.at(Team);
			cout << "Detected team change : to " << teamname <<endl; 
			LastTeam = Team;
		}
		
		prof.EnterSection(ps++);
		int64 GrabTick = getTickCount();
		ObjectTracker* TrackerToUse = &bluetracker;
		if (Team == CDFRTeam::Green)
		{
			TrackerToUse = &greentracker;
		}
		 
		BufferedPipeline(0, vector<ArucoCamera*>(physicalCameras.begin(), physicalCameras.end()), Detector, TrackerToUse);
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
		TrackerToUse->SolveLocationsPerObject(arucoDatas, GrabTick);
		vector<ObjectData> ObjData = TrackerToUse->GetObjectDataVector(GrabTick);
		ObjectData TeamPacket(ObjectIdentity(PacketType::Team, (int)Team));
		ObjData.insert(ObjData.begin(), TeamPacket); //insert team as the first object
		//Vec3d diff = robot1->GetLocation().translation() - robot2->GetLocation().translation(); 
		//cout << "Robot 1 location : " << robot1->GetLocation().translation() << endl;
		//cout << "Distance robot 1-2: " << sqrt(diff.ddot(diff)) << " m" <<endl;

		prof.EnterSection(ps++);
		
		if (v3d)
		{
			if(!OpenGLBoard.Tick(ObjectData::ToGLObjects(ObjData)))
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
		logger.SendPacket(GrabTick, ObjData);
		
		
		prof.EnterSection(-1);
		
		
		
		/*if (prof.ShouldPrint())
		{
			cout << fps.GetFPSString(deltaTime) << endl;
			prof.PrintIfShould();
		}	*/
		
	}
}