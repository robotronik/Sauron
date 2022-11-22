#pragma once

#include <vector>
#include <string>
#include <iostream>

#include "Cameras/Camera.hpp"
#include "GlobalConf.hpp"

class CameraManager
{
private:
	CameraStartType Start;
	std::string Filter;
	bool AllowNoCalib;
	int scanidx;
	std::vector<std::string> paths;
public:
	std::function<bool(CameraSettings)> OnConnect;
	std::function<bool(ArucoCamera*)> PostCameraConnect, OnDisconnect;
	std::vector<ArucoCamera*> Cameras;

	CameraManager(CameraStartType InStart, std::string InFilter, bool InAllowNoCalib = false)
		:Start(InStart), Filter(InFilter), AllowNoCalib(InAllowNoCalib),
		scanidx(0)
	{

	}
	~CameraManager()
	{
		for (int i = 0; i < Cameras.size(); i++)
		{
			delete Cameras[i];
		}
		
	}

	static bool DeviceInFilter(v4l2::devices::DEVICE_INFO device, std::string Filter);

	static CameraSettings DeviceToSettings(v4l2::devices::DEVICE_INFO device, CameraStartType Start);

	static std::vector<CameraSettings> autoDetectCameras(CameraStartType Start, std::string Filter, std::string CalibrationFile, bool silent = true);

private:
	template<class CameraType>
	CameraType* StartCamera(CameraSettings Settings)
	{
		ArucoCamera* cam = new CameraType(Settings);
		if(!cam->StartFeed())
		{
			std::cerr << "ERROR! Unable to open camera " << cam->GetCameraSettings().DeviceInfo.device_description << std::endl;
			delete cam;
			return nullptr;
		}
		Cameras.push_back(cam);
		paths.push_back(Settings.DeviceInfo.device_paths[0]);
		
		return (CameraType*) cam;
	}

public:
	template<class CameraType>
	void Tick()
	{
		for (int i = 0; i < Cameras.size(); i++)
		{
			if (Cameras[i]->errors >= 20)
			{
				std::cerr << "Detaching camera @" << Cameras[i]->GetCameraSettings().DeviceInfo.device_paths[0] << std::endl;
				std::string pathtofind = Cameras[i]->GetCameraSettings().DeviceInfo.device_paths[0];
				if (OnDisconnect)
				{
					if (!OnDisconnect(Cameras[i]))
					{
						continue;
					}
				}
				
				auto pos = std::find(paths.begin(), paths.end(), pathtofind);
				if (pos != paths.end())
				{
					paths.erase(pos);
				}
				delete Cameras[i];
				Cameras.erase(std::next(Cameras.begin(), i));
				i--;
			}
		}
		if (scanidx == 0)
		{
			std::vector<v4l2::devices::DEVICE_INFO> devices;
			v4l2::devices::list(devices);
			for (int i = 0; i < devices.size(); i++)
			{
				if ((devices[i].device_paths.size() > 0) && DeviceInFilter(devices[i], Filter))
				{
					std::string pathtofind = devices[i].device_paths[0];
					auto pos = std::find(paths.begin(), paths.end(), pathtofind);
					if (pos == paths.end()) //new camera
					{
						std::cerr << "Detected camera " << devices[i].device_description << " @" << devices[i].device_paths[0] << std::endl;
						CameraSettings settings = DeviceToSettings(devices[i], Start);
						if (AllowNoCalib || settings.CameraMatrix.size() == cv::Size(3,3))
						{
							if (OnConnect)
							{
								if (!OnConnect(settings))
								{
									continue;
								}
								
							}
							
							ArucoCamera* cam = StartCamera<CameraType>(settings);
							std::vector<InternalCameraConfig>& campos = GetInternalCameraPositionsConfig();
							for (int j = 0; j < campos.size(); j++)
							{
								if (DeviceInFilter(devices[i], campos[j].CameraName))
								{
									cam->SetLocation(campos[j].LocationRelative);
									std::cout << "Using stored position " << j << " for camera " << devices[i].device_description << std::endl;
									break;
								}
								
							}
							if (PostCameraConnect)
							{
								PostCameraConnect(cam);
							}
							
							
						}
					}
				}
			}
		}
		scanidx = (scanidx + 1) & 15;
	}
};
