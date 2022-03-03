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

#include "GlobalConf.hpp"
#include "OutputImage.hpp"
#include "Camera.hpp"
#include "trackedobject.hpp"
#include "Calibrate.hpp"
#include "boardviz.hpp"
#include "BoardViz3d.hpp"
#include "FrameCounter.hpp"
using namespace std;
using namespace cv;

vector<Camera*> physicalCameras;

void GrabReadCameras(vector<Camera*> Cameras, Ptr<aruco::Dictionary> dict, Ptr<aruco::DetectorParameters> params)
{
	for (int i = 0; i < Cameras.size(); i++)
	{
		Cameras[i]->Grab(0);
	}
	parallel_for_(Range(0, Cameras.size()), [&](const Range& range)
	{
		for (int i = range.start; i < range.end; i++)
		{
			if(Cameras[i]->Read(0))
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
			Cameras[i]->detectMarkers(0, dict, params);
		}
		//cout << "Aruco stripe from " << range.start << " to " << range.end << endl;
	});
}

void BufferedPipeline(int BufferCaptureIdx, vector<Camera*> Cameras, Ptr<aruco::Dictionary> dict, Ptr<aruco::DetectorParameters> params)
{
	int numCams = Cameras.size();
	for (int i = 0; i < Cameras.size(); i++)
	{
		Cameras[i]->Grab(BufferCaptureIdx);
	}
	parallel_for_(Range(0, numCams * Camera::FrameBufferSize), [&](const Range& range)
	{
		for (int i = range.start; i < range.end; i++)
		{
			int BufferIdx = (BufferCaptureIdx + i/numCams) % Camera::FrameBufferSize;
			switch (i/numCams)
			{
			case 0:
				// read
				Cameras[i%numCams]->Read(BufferIdx);
				break;
			
			case 1:
				//Detect aruco
				Cameras[i%numCams]->detectMarkers(BufferIdx, dict, params);
				break;

			default:
				cout << "Frame buffer too big or operation unimplemented" << endl;
				break;
			}
			
			
		}
		//cout << "Aruco stripe from " << range.start << " to " << range.end << endl;
	}, numCams*Camera::FrameBufferSize);
} 

UMat ConcatCameras(int BufferIndex, vector<OutputImage*> Cameras, int NumCams)
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
			/*UMat frame; Cameras[i]->GetFrame(BufferIndex, frame);
			if (frame.empty())
			{
				continue;
			}*/
			UMat region = concatenated(roi);
			Cameras[i]->GetOutputFrame(BufferIndex, region, Size(winWidth, winHeight));
		}
	});
	return concatenated;
}

int main(int argc, char** argv )
{
	const string keys = 
		"{help h usage ? |      | print this message   }"
		"{direct d       |      | show direct camera output }"
		"{build b        |      | print build information   }"
		"{calibrate c    |      | start camera calibration wizard   }"
		"{marker m       |      | print out markers               }"
		"{cuda           |      | print cuda info         }"
		"{board          |      | runs boardview test }"
		"{viz3d          |      | runs viz3d test }"
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
	if (parser.has("viz3d"))
	{
		TestViz3D();
		exit(EXIT_SUCCESS);
	}
	Ptr<aruco::Dictionary> dictionary = GetArucoDict();
	if (parser.has("marker"))
	{
		cout << "markers folder must exist before calling this function" << endl;
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
	boardviz* board = new boardviz(FVector2D<float>(3.0f, 2.0f), FVector2D<float>(1.5f, 1.0f));
	if (direct)
	{
		boardviz::InitImages();
	}
	
	
	viz::Viz3d board3d("3D board");
	BoardViz3D::SetupTerrain(board3d);

	Tracker3DTest();
	int lastmarker = 0;
	for (;;)
	{
		fpsPipeline.GetDeltaTime();
		BufferedPipeline(PipelineIdx, physicalCameras, dictionary, parameters);
		PipelineIdx = (PipelineIdx + 1) % Camera::FrameBufferSize;
		double TimePipeline = fpsPipeline.GetDeltaTime();

		//cout << "Pipeline took " << TimePipeline << "s to run" << endl;

		if (direct)
		{
			board->CreateBackground(Size(1500, 1000));
			board->OverlayImage(board->GetPalet(PaletCouleur::autre), FVector2D<float>(1), 0, FVector2D<float>(0.1));
		}
		
		

		for (int i = 0; i < lastmarker; i++)
		{
			board3d.removeWidget(String("marker") + to_string(i));
		}
		lastmarker = 0;

		for (int i = 0; i < physicalCameras.size(); i++)
		{
			Camera* cam = physicalCameras[i];
			vector<int> MarkerIDs; vector<vector<Point2f>> MarkerCorners;
			if (!cam->GetMarkerData(PipelineIdx, MarkerIDs, MarkerCorners))
			{
				continue;
			}
			Affine3d CamTransform = Affine3d::Identity();
			bool has42 = false;
			for (int mark = 0; mark < MarkerIDs.size(); mark++)
			{
				int markerid = MarkerIDs[mark];
				switch (markerid)
				{
				case 42:
					{
						CamTransform = GetTransformRelativeToTag(center, MarkerCorners[mark], cam);
						cam->Location = CamTransform;
						has42 = true;
					}
					break;
				
				default:
					break;
				}
			}
			BoardViz3D::ShowCamera(board3d, cam, PipelineIdx, cam->Location, has42 ? viz::Color::green() : viz::Color::red());
			for (int mark = 0; mark < MarkerIDs.size(); mark++)
			{
				int markerid = MarkerIDs[mark];
				switch (markerid)
				{
				case 42:
					continue;
				
				default:
					ArucoMarker markerstruct(0.05, markerid);
					Affine3d MarkerTransform = cam->Location * GetTagTransform(markerstruct, MarkerCorners[mark], cam);
					viz::WImage3D markerWidget(GetArucoImage(markerid), Size2d(0.05, 0.05));
					board3d.showWidget(String("marker") + to_string(lastmarker++), markerWidget, MarkerTransform);
					break;
				}
			}
			
		}
		

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
			//board.GetOutputFrame(0, image, Size(1920,1080));
			//cout << "Concat OK" <<endl;
			fps.AddFpsToImage(image, deltaTime);
			//printf("fps : %f\n", fps);
			imshow("Cameras", image);
		}
		
		
		viz::WText fpstext(to_string(1/deltaTime), Point2i(100,100));
		board3d.showWidget("fps", fpstext);
		board3d.spinOnce(1, true);
		if (waitKey(1) == 27 || board3d.wasStopped())
			break;
		
	}
	// the camera will be deinitialized automatically in VideoCapture destructor
	
	return 0;
}