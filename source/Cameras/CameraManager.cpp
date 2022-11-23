#include "Cameras/CameraManager.hpp"

#include "GlobalConf.hpp"
#include "data/Calibfile.hpp"

#include <filesystem>


using namespace std;

bool CameraManager::DeviceInFilter(v4l2::devices::DEVICE_INFO device, std::string Filter)
{
	bool invertedFilter = false;
	if (Filter.length() > 1 && Filter[0] == '!')
	{
		invertedFilter = true;
		Filter = Filter.substr(1);
	}
	if (Filter.length() == 0)
	{
		return true;
	}
	
	return (device.device_description.find(Filter) != string::npos) ^ invertedFilter;
}

CameraSettings CameraManager::DeviceToSettings(v4l2::devices::DEVICE_INFO device, CameraStartType Start)
{
	CameraSettings settings;
	CaptureConfig cfg = GetCaptureConfig();
	settings.Resolution = cfg.FrameSize;
	settings.Framerate = cfg.CaptureFramerate;
	settings.FramerateDivider = cfg.FramerateDivider;
	settings.DeviceInfo = device;
	settings.BufferSize = 2;
	settings.StartType = Start;

	//these only get populated when StartFeed is called
	settings.StartPath = "";
	settings.ApiID = -1;

	string CalibrationRoot = string("../calibration/");
	string CalibrationPath = CalibrationRoot + settings.DeviceInfo.device_description;

	/*if (!filesystem::exists(CalibrationPath))
	{
		return settings;
	}*/
	

	readCameraParameters(CalibrationPath, settings.CameraMatrix, settings.distanceCoeffs, settings.Resolution);
	//cout << "Camera matrix : " << cam->CameraMatrix << " / Distance coeffs : " << cam->distanceCoeffs << endl;
	return settings;
}

vector<CameraSettings> CameraManager::autoDetectCameras(CameraStartType Start, string Filter, string CalibrationFile, bool silent)
{
	vector<v4l2::devices::DEVICE_INFO> devices;

	v4l2::devices::list(devices);

	if (!silent)
	{
		for (const auto & device : devices) 
		{
			cout << device.device_description <<  " at " << device.bus_info << " is attached to\n";
			for (const auto & path : device.device_paths) {
				cout << path << "\n";
			}
		}
	}
	vector<CameraSettings> detected;
	for (const auto & device : devices)
	{
		//v4l2-ctl --list-formats-ext
		//gst-launch-1.0 v4l2src device="/dev/video0" io-mode=2 ! "image/jpeg, width=1920, height=1080, framerate=30/1" ! nvjpegdec ! "video/x-raw" ! nvoverlaysink -e
		//gst-launch-1.0 v4l2src device="/dev/video0" ! "video/x-h264, format=H264, width=1920, height=1080, framerate=30/1" ! h264parse ! omxh264dec ! nvvidconv ! "video/x-raw(memory:NVMM), format=NV12" ! nvoverlaysink -e
		
		//gst-launch-1.0 v4l2src device="/dev/video0" io-mode=2 ! "image/jpeg, width=1920, height=1080, framerate=60/1" ! nvdec ! glimagesink -e
		
		//nvv4l2decoder ?
		//string capname = string("v4l2src device=/dev/video") + to_string(i)
		//+ String(" io-mode=2 do-timestamp=true ! image/jpeg, width=1920, height=1080, framerate=30/2 ! nvv4l2decoder mjpeg=1 ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink");
		
		//jpegdec + videoconvert marche
		//nvdec ! glcolorconvert ! gldownload
		if (DeviceInFilter(device, Filter))
		{
			detected.push_back(DeviceToSettings(device, Start));
		}
	}
	return detected;
}
