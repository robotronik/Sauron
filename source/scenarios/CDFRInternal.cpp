#include "Scenarios/CDFRInternal.hpp"
#include "Scenarios/CDFRCommon.hpp"

#include "visualisation/BoardGL.hpp"
#include "data/senders/Encoders/MinimalEncoder.hpp"
#include "data/senders/Transport/TCPTransport.hpp"
#include "data/senders/Transport/UDPTransport.hpp"
#include "data/senders/Transport/SerialTransport.hpp"

void CDFRInternalMain(bool direct, bool v3d)
{

	CameraManager CameraMan(GetCaptureMethod(), GetCaptureConfig().filter, false);

	Ptr<aruco::Dictionary> dictionary = GetArucoDict();

	vector<ArucoCamera*> &physicalCameras = CameraMan.Cameras;
	if (direct)
	{
		namedWindow("Cameras", WINDOW_NORMAL);
		setWindowProperty("Cameras", WND_PROP_FULLSCREEN, WINDOW_FULLSCREEN);
	}
	
	#ifdef WITH_VTK
	viz::Viz3d viz3dhandle("local robot");
	BoardViz3D board3d(&viz3dhandle);
	if (v3d)
	{
		BoardViz3D::SetupRobot(viz3dhandle);
	}
	#endif

	BoardGL OpenGLBoard;
	OpenGLBoard.Start();
	
	Ptr<aruco::DetectorParameters> parameters = GetArucoParams();
	FrameCounter fps;

	ObjectTracker tracker;
	
	PositionDataSender sender;
	{
		WebsocketConfig wscfg = GetWebsocketConfig();
		sender.encoder = new MinimalEncoder;
		sender.transport = new SerialTransport(115200, true);
		sender.StartReceiveThread();
	}

	StaticObject* boardobj = new StaticObject(true, "board");
	tracker.RegisterTrackedObject(boardobj);

	TrackerCube* robot1 = new TrackerCube({51, 52, 54, 55}, 0.06, Point3d(0.0952, 0.0952, 0), "Robot1");
	TrackerCube* robot2 = new TrackerCube({57, 58, 59, 61}, 0.06, Point3d(0.0952, 0.0952, 0), "Robot2");
	tracker.RegisterTrackedObject(robot1);
	tracker.RegisterTrackedObject(robot2);
	
	int hassent = 0;
	for (;;)
	{
		CameraMan.Tick<VideoCaptureCamera>();
		BufferedPipeline(0, vector<ArucoCamera*>(physicalCameras.begin(), physicalCameras.end()), dictionary, parameters, &tracker);

		//cout << "Pipeline took " << TimePipeline << "s to run" << endl;
		
		vector<CameraArucoData> arucoDatas;
		int NumCams = physicalCameras.size();
		arucoDatas.resize(NumCams);

		for (int i = 0; i < physicalCameras.size(); i++)
		{
			ArucoCamera* cam = physicalCameras[i];
			if (!cam->GetMarkerData(0, arucoDatas[i]))
			{
				continue;
			}
		}
		
		tracker.SolveLocationsPerObject(arucoDatas);
		vector<ObjectData> ObjData = tracker.GetObjectDataVector();
		double deltaTime = fps.GetDeltaTime();

		#ifdef WITH_VTK
		if (v3d)
		{
			if (viz3dhandle.wasStopped())
			{
				break;
			}
			board3d.DisplayData(ObjData);

			viz::WText fpstext(to_string(1/deltaTime), Point2i(200,100));
			viz3dhandle.showWidget("fps", fpstext);
			viz3dhandle.spinOnce(1, true);
		}
		#endif
		
		if (direct)
		{
			vector<OutputImage*> OutputTargets;
			for (int i = 0; i < physicalCameras.size(); i++)
			{
				OutputTargets.push_back(physicalCameras[i]);
			}
			UMat image = ConcatCameras(0, OutputTargets, OutputTargets.size());
			//board.GetOutputFrame(0, image, GetFrameSize());
			//cout << "Concat OK" <<endl;
			fps.AddFpsToImage(image, deltaTime);
			//printf("fps : %f\n", fps);
			imshow("Cameras", image);
		}

		if (!OpenGLBoard.Tick(ObjData))
		{
			break;
		}

		if (waitKey(1) == '\e')
		{
			break;
		}
	}
}