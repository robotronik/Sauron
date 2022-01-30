#include "Camera.hpp"


#include <iostream> // for standard I/O
#include <iomanip>  // for controlling float print precision
#include <sstream>  // string to number conversion
#include <opencv2/imgproc.hpp>
#include "list-devices.hpp"
#include "Calibfile.hpp"

using namespace std;

vector<Camera*> autoDetectCameras()
{
	vector<v4l2::devices::DEVICE_INFO> devices;

    v4l2::devices::list(devices);

    for (const auto & device : devices) 
    {
    
        cout << device.device_description <<  " at " << device.bus_info << " is attached to\n";

        for (const auto & path : device.device_paths) {
            cout << path << "\n";
        }
        
    }

    vector<Camera*> detected;
	for (const auto & device : devices)
	{
		//v4l2-ctl --list-formats-ext
		//gst-launch-1.0 v4l2src device="/dev/video0" io-mode=2 ! "image/jpeg, width=1920, height=1080, framerate=30/1" ! nvjpegdec ! "video/x-raw" ! nvoverlaysink -e
		//gst-launch-1.0 v4l2src device="/dev/video0" ! "video/x-h264, format=H264, width=1920, height=1080, framerate=30/1" ! h264parse ! omxh264dec ! nvvidconv ! "video/x-raw(memory:NVMM), format=NV12" ! nvoverlaysink -e
		//nvv4l2decoder ?
		//string capname = string("v4l2src device=/dev/video") + to_string(i)
		//+ String(" io-mode=2 do-timestamp=true ! image/jpeg, width=1920, height=1080, framerate=30/2 ! nvv4l2decoder mjpeg=1 ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink");
		
		if (strstr(device.device_description.c_str(), "C920") != NULL)
		{
			string capname = String(device.device_paths[0]);
			int API = CAP_ANY;
			Camera* cam = new Camera(device.device_description, Size(1920, 1080), 30, capname, API, true);
			//cam->StartFeed();
			readCameraParameters("c920", cam->CameraMatrix, cam->distanceCoeffs);
			detected.push_back(cam);
		}
	}
    return detected;
}

bool StartCameras(vector<Camera*> Cameras)
{
	for (int i = 0; i < Cameras.size(); i++)
	{
		if(!Cameras[i]->StartFeed())
		{
			cout << "ERROR! Unable to open camera " << Cameras[i]->WindowName;
		}
	}
	return true;
}