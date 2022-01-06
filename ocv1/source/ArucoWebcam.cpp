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
#include "Camera.hpp"
#include "trackedobject.hpp"
using namespace std;
using namespace cv;


//Camera cam0 = Camera("Camera0", Size(1920, 1080), 30, "/dev/video0", CAP_ANY);
//Camera cam1 = Camera("Camera1", Size(1920, 1080), 30, "/dev/video2", CAP_ANY);
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

/*void DisplayCameras(vector<Camera*> Cameras)
{
	for (int i = 0; i < Cameras.size(); i++)
	{
		Cameras[i]->Show();
		//cout << "shown camera " << i<< endl;
	}
}*/

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
			if (Cameras[i]->frame.empty())
			{
				continue;
			}
			Cameras[i]->GetFrameDebug(Size(winWidth, winHeight)).copyTo(concatenated(roi));
		}
	});
	return concatenated;
}

vector<Camera*> Cameras;

int main(int argc, char** argv )
{
    /*int cuda_devices_number = getCudaEnabledDeviceCount();
    cout << "CUDA Device(s) Number: "<< cuda_devices_number << endl;
	if (cuda_devices_number > 0)
	{
		DeviceInfo _deviceInfo;
		bool _isd_evice_compatible = _deviceInfo.isCompatible();
		cout << "CUDA Device(s) Compatible: " << _isd_evice_compatible << endl;
		printShortCudaDeviceInfo(getDevice());
	}*/
	physicalCameras = autoDetectCameras(10);
	for (int i = 0; i < physicalCameras.size(); i++)
	{
		Cameras.push_back(physicalCameras[i]);
		/*Camera* arucopair = new Camera(physicalCameras[i]->WindowName + string("Aruco"));
		virtualCameras.push_back(arucopair);
		Cameras.push_back(arucopair);*/
	}
	
	namedWindow("Cameras", WINDOW_NORMAL);
	setWindowProperty("Cameras", WND_PROP_FULLSCREEN, WINDOW_FULLSCREEN);

	StartCameras(physicalCameras);

	cout << "Start grabbing " << Cameras.size() << " cameras, " << physicalCameras.size() << " physical" << endl
		<< "Press any key to terminate" << endl;

	
	Ptr<aruco::DetectorParameters> parameters = aruco::DetectorParameters::create();
	parameters->cornerRefinementMethod = aruco::CORNER_REFINE_CONTOUR;
	Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_100);
	int64 dt = 0;
	for (;;)
	{
		GrabReadCameras(physicalCameras, dictionary, parameters);
		DetectArucoCameras(physicalCameras, dictionary, parameters);

		for (int i = 0; i < Cameras.size(); i++)
		{
			Camera* cam = Cameras[i];
			if (cam->arucoed)
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
		

		int64 dt2 = getTickCount();
		double fps = getTickFrequency() / (double)(dt2-dt);
		dt = dt2;
		UMat image = ConcatCameras(Cameras, Cameras.size());
		String strfps = String("fps : ") + to_string(fps);
		putText(image, strfps, Point2i(0,image.rows-20), FONT_HERSHEY_SIMPLEX, 2, Scalar(255, 255, 255), 5);
		putText(image, strfps, Point2i(0,image.rows-20), FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 0, 0), 2);
		imshow("Cameras", image);
		//DisplayCameras(Cameras);
		if (waitKey(5) >= 0)
			break;
		
	}
	// the camera will be deinitialized automatically in VideoCapture destructor
	
	return 0;
}