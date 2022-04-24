#include "Camera.hpp"


#include <iostream> // for standard I/O
#include <iomanip>  // for controlling float print precision
#include <sstream>  // string to number conversion
#include <opencv2/imgproc.hpp>
#include <opencv2/cudacodec.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/calib3d.hpp>
#include "thirdparty/list-devices.hpp"
//#include "thirdparty/vidcap.h"
#include "data/Calibfile.hpp"
#include "TrackedObjects/TrackedObject.hpp" //CameraView
#include "ObjectTracker.hpp"
#include "data/FrameCounter.hpp"
#include "GlobalConf.hpp"
#include "data/FVector2D.hpp"

using namespace std;

bool BufferedFrame::GetCPUFrame(UMat& OutFrame)
{
	switch (Status)
	{
	case CameraStatus::ReadGPU :
		GPUFrame.download(CPUFrame);
		Status = CameraStatus::ReadBoth;
	case CameraStatus::ReadCPU :
	case CameraStatus::ReadBoth :
		OutFrame = CPUFrame;
		return true;
	default:
		return false;
		break;
	}
}

bool BufferedFrame::GetGPUFrame(cuda::GpuMat& OutFrame)
{
	switch (Status)
	{
	case CameraStatus::ReadCPU :
		GPUFrame.upload(CPUFrame);
		Status = CameraStatus::ReadBoth;
	case CameraStatus::ReadGPU :
	case CameraStatus::ReadBoth :
		OutFrame = GPUFrame;
		return true;
	default:
		return false;
		break;
	}
}

bool BufferedFrame::GetRescaledFrame(int index, UMat& OutFrame)
{
	if (0 <= index && index < rescaledFrames.size())
	{
		OutFrame = rescaledFrames[index];
		return true;
	}
	return false;
}

CameraSettings Camera::GetCameraSettings()
{
	return Settings;
}

bool Camera::SetCameraSetting(CameraSettings InSettings)
{
	if (connected)
	{
		cerr << "WARNING: Tried to set camera settings, but camera is already started ! Settings have not been applied." << endl;
		return false;
	}
	
}

CameraStatus Camera::GetStatus(int BufferIndex)
{
	return FrameBuffer[BufferIndex].Status;
}

bool Camera::StartFeed()
{
	if (connected)
	{
		return false;
	}
	FrameBuffer.resize(Settings.BufferSize);
	for (int i = 0; i < Settings.BufferSize; i++)
	{
		FrameBuffer[i] = BufferedFrame();
	}
	
	if (Settings.StartType == CameraStartType::CUDA)
	{
		Settings.StartPath = Settings.DeviceInfo.device_paths[0];
		Settings.ApiID = CAP_ANY;

		d_reader = cudacodec::createVideoReader(Settings.DeviceInfo.device_paths[0], {
			CAP_PROP_FRAME_WIDTH, Settings.Resolution.width, 
			CAP_PROP_FRAME_HEIGHT, Settings.Resolution.height, 
			CAP_PROP_FPS, Settings.Framerate/Settings.FramerateDivider,
			CAP_PROP_BUFFERSIZE, 1});

	}
	else
	{
		switch (Settings.StartType)
		{
		case CameraStartType::GSTREAMER_CPU:
		case CameraStartType::GSTREAMER_NVDEC:
		case CameraStartType::GSTREAMER_JETSON:
			{
				ostringstream capnamestream;
				capnamestream << "v4l2src device=" << Settings.DeviceInfo.device_paths[0] << " io-mode=4 ! image/jpeg, width=" 
				<< Settings.Resolution.width << ", height=" << Settings.Resolution.height << ", framerate="
				<< Settings.Framerate << "/" << Settings.FramerateDivider << ", num-buffers=1 ! ";
				if (Settings.StartType == CameraStartType::GSTREAMER_CPU)
				{
					capnamestream << "jpegdec ! videoconvert ! ";
				}
				else if (Settings.StartType == CameraStartType::GSTREAMER_JETSON)
				{
					capnamestream << "nvv4l2decoder mjpeg=1 ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! ";
				}
				else if(Settings.StartType == CameraStartType::CUDA)
				{
					capnamestream << "nvdec ! glcolorconvert ! gldownload ! ";
				}
				capnamestream << "video/x-raw, format=BGR ! appsink";
				Settings.StartPath = capnamestream.str();
				Settings.ApiID = CAP_GSTREAMER;
			}
			break;
		case CameraStartType::CUDA:
		case CameraStartType::ANY:
		default:
			Settings.StartPath = Settings.DeviceInfo.device_paths[0];
			Settings.ApiID = CAP_ANY;
			break;
		}

		feed = new VideoCapture();
		cout << "Opening device at \"" << Settings.StartPath << "\" with API id " << Settings.ApiID << endl;
		feed->open(Settings.StartPath, Settings.ApiID);
		if (Settings.StartType == CameraStartType::ANY)
		{
			feed->set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
			//feed->set(CAP_PROP_FOURCC, VideoWriter::fourcc('Y', 'U', 'Y', 'V'));
			feed->set(CAP_PROP_FRAME_WIDTH, Settings.Resolution.width);
			feed->set(CAP_PROP_FRAME_HEIGHT, Settings.Resolution.height);
			feed->set(CAP_PROP_FPS, Settings.Framerate/Settings.FramerateDivider);
			feed->set(CAP_PROP_BUFFERSIZE, 1);
		}
		//cout << "Success opening ? " << feed->isOpened() << endl;
		/*if (!feed->isOpened())
		{
			exit(EXIT_FAILURE);
		}*/
		
	}
	
	connected = true;
	return true;
}

bool Camera::Grab(int BufferIndex)
{
	if (!connected)
	{
		
		return false;
	}
	bool grabsuccess = false;
	if (Settings.StartType == CameraStartType::CUDA)
	{
		grabsuccess = d_reader->grab();
	}
	else
	{
		grabsuccess = feed->grab();
	}
	if (grabsuccess)
	{
		FrameBuffer[BufferIndex].Status = (Settings.StartType == CameraStartType::CUDA) ? CameraStatus::GrabbedGPU : CameraStatus::GrabbedCPU;
	}
	else
	{
		cerr << "Failed to grab frame for camera " << Settings.DeviceInfo.device_description << " with buffer " << BufferIndex <<endl;
		FrameBuffer[BufferIndex].Status = CameraStatus::None;
	}
	
	return grabsuccess;
}

bool Camera::Read(int BufferIndex)
{
	BufferedFrame& buff = FrameBuffer[BufferIndex];
	if (!connected)
	{
		return false;
	}
	bool ReadSuccess = false;
	if (Settings.StartType == CameraStartType::CUDA)
	{
		if (buff.Status == CameraStatus::GrabbedGPU)
		{
			ReadSuccess = d_reader->retrieve(buff.GPUFrame);
		}
		else
		{
			ReadSuccess = d_reader->nextFrame(buff.GPUFrame);
		}
	}
	else
	{
		if (buff.Status == CameraStatus::GrabbedCPU)
		{
			ReadSuccess = feed->retrieve(buff.CPUFrame);
		}
		else
		{
			ReadSuccess = feed->read(buff.CPUFrame);
		}
	}
	if (ReadSuccess)
	{
		buff.Status = (Settings.StartType == CameraStartType::CUDA) ? CameraStatus::ReadGPU : CameraStatus::ReadCPU;
		buff.HasAruco = false;
		buff.HasViews = false;
	}
	else
	{
		cerr << "Failed to read frame for camera " << Settings.DeviceInfo.device_description << " with buffer " << BufferIndex <<endl;
		buff.Status = CameraStatus::None;
		buff.rescaledFrames.resize(0);
		return false;
	}
	
	return true;
}

void Camera::RescaleFrames(int BufferIdx)
{
	BufferedFrame& buff = FrameBuffer[BufferIdx];
	vector<Size> rescales = GetArucoReductions();
	buff.rescaledFrames.resize(rescales.size());
	bool GPUconvert = false;
	cuda::GpuMat GPUFrame; UMat CPUFrame;
	if (GPUconvert)
	{
		buff.GetGPUFrame(GPUFrame);
	}
	else
	{
		buff.GetCPUFrame(CPUFrame);
		
	}
	
	
	for (int i = 0; i < rescales.size(); i++)
	{
		if (GPUconvert)
		{
			cuda::GpuMat rescaled, gray;
			cuda::resize(GPUFrame, rescaled, rescales[i], 0, 0, INTER_AREA);
			cuda::cvtColor(rescaled, gray, COLOR_BGR2GRAY);
			gray.download(buff.rescaledFrames[i]);
		}
		else
		{
			UMat rescaled, hsv;
			resize(CPUFrame, rescaled, rescales[i], 0, 0, INTER_AREA);
			cvtColor(rescaled, buff.rescaledFrames[i], COLOR_BGR2GRAY);
		}
	}
}

void Camera::GetFrame(int BufferIndex, UMat& OutFrame)
{
	FrameBuffer[BufferIndex].GetCPUFrame(OutFrame);
}

void Camera::GetOutputFrame(int BufferIndex, UMat& OutFrame, Size winsize)
{
	UMat frame;
	if (!FrameBuffer[BufferIndex].GetCPUFrame(frame))
	{
		return;
	}

	double fx = frame.cols / winsize.width;
	double fy = frame.rows / winsize.height;
	double fz = max(fx, fy);

	resize(frame, OutFrame, Size(winsize.width, winsize.height), fz, fz, INTER_LINEAR);
	//cout << "Resize OK" <<endl;
	if (FrameBuffer[BufferIndex].HasAruco)
	{
		vector<vector<Point2f>> raruco;
		for (int i = 0; i < FrameBuffer[BufferIndex].markerCorners.size(); i++)
		{
			vector<Point2f> marker;
			for (int j = 0; j < FrameBuffer[BufferIndex].markerCorners[i].size(); j++)
			{
				marker.push_back(FrameBuffer[BufferIndex].markerCorners[i][j]/fz);
			}
			raruco.push_back(marker);
		}
		aruco::drawDetectedMarkers(OutFrame, raruco, FrameBuffer[BufferIndex].markerIDs);
	}
}

void Camera::detectMarkers(int BufferIndex, Ptr<aruco::Dictionary> dict, Ptr<aruco::DetectorParameters> params)
{
	/*cuda::GpuMat framegpu; framegpu.upload(frame);
	cuda::cvtColor(framegpu, framegpu, COLOR_BGR2GRAY);
	cuda::threshold(framegpu, framegpu, 127, 255, THRESH_BINARY);
	UMat framethreshed; framegpu.download(framethreshed);*/

	BufferedFrame& buff = FrameBuffer[BufferIndex];

	vector<vector<vector<Point2f>>> AllCorners;
	vector<vector<int>> AllIDs;
	vector<vector<FVector2D<double>>> Centers;
	int nbframes = buff.rescaledFrames.size();
	AllCorners.resize(nbframes);
	AllIDs.resize(nbframes);
	Centers.resize(nbframes);
	Size framesize = GetFrameSize();
	vector<Size> rescaled = GetArucoReductions();

	UMat framebase, framegray;
	if (!buff.GetCPUFrame(framebase))
	{
		return;
	}
	cvtColor(framebase, framegray, COLOR_BGR2GRAY);
	
	

	//Range InRange(0, nbframes);
	parallel_for_(Range(0,nbframes), [&](const Range InRange)
	{
		for (size_t i = InRange.start; i < InRange.end; i++)
		{
			vector<vector<Point2f>> corners;
			vector<int>& IDs = AllIDs[i];
			UMat frame;
			if (!buff.GetRescaledFrame(i, frame))
			{
				continue;
			}
			aruco::detectMarkers(frame, dict, corners, IDs, params);

			//cout << "Height " << i << " found " << corners.size() << " arucos" << endl;

			AllCorners[i].resize(corners.size());
			Centers[i].resize(corners.size());

			FVector2D scalefactor = FVector2D(framesize)/FVector2D(rescaled[i]);

			for (int j = 0; j < corners.size(); j++)
			{
				FVector2D<double> sum = 0;
				AllCorners[i][j].resize(4);
				for (size_t k = 0; k < 4; k++)
				{
					FVector2D pos = FVector2D(corners[j][k]) * scalefactor;
					AllCorners[i][j][k] = pos;
					sum = sum + pos;
				}
				Centers[i][j] = sum /4;
			}
			
		}
	});

	vector<vector<Point2f>> corners;
	vector<int> markerIDs;
	vector<float> reductionFactors = GetReductionFactor();

	for (int Lower = 0; Lower < nbframes; Lower++)
	{
		for (int ArucoLower = 0; ArucoLower < AllIDs[Lower].size(); ArucoLower++)
		{
			markerIDs.push_back(AllIDs[Lower][ArucoLower]);
			vector<Point2f> cornersTemp = AllCorners[Lower][ArucoLower];
			Size window = Size(reductionFactors[Lower], reductionFactors[Lower]);
			//cornerSubPix(framegray, cornersTemp, window, Size(-1,-1), TermCriteria(TermCriteria::COUNT | TermCriteria::EPS, 100, 0.01));
			corners.push_back(cornersTemp);
			//cout << "Found aruco " << AllIDs[Lower][ArucoLower] << " at height " << Lower << endl;

			for (int Higher = Lower +1; Higher < nbframes; Higher++)
			{
				for (int ArucoHigher = 0; ArucoHigher < AllIDs[Higher].size(); ArucoHigher++)
				{
					if (AllIDs[Lower][ArucoLower] == AllIDs[Higher][ArucoHigher])
					{
						if ((Centers[Lower][ArucoLower] - Centers[Higher][ArucoHigher]).TaxicabLength() < 10)
						{
							Centers[Higher].erase(Centers[Higher].begin() + ArucoHigher);
							AllIDs[Higher].erase(AllIDs[Higher].begin() + ArucoHigher);
							AllCorners[Higher].erase(AllCorners[Higher].begin() + ArucoHigher);
							break;
							ArucoHigher--;
						}
						
					}
					
				}
				
			}
			
		}
		
	}
	
	buff.markerCorners = corners;
	buff.markerIDs = markerIDs;

	
	
	buff.HasAruco = true;
}

void Camera::SolveMarkers(int BufferIndex, int CameraIdx, ObjectTracker* registry)
{
	BufferedFrame& buff = FrameBuffer[BufferIndex];
	if (!buff.HasAruco)
	{
		return;
	}
	buff.markerViews.resize(buff.markerIDs.size());
	for (int i = 0; i < buff.markerIDs.size(); i++)
	{
		float sideLength = registry->GetArucoSize(buff.markerIDs[i]);
		Affine3d TagTransform = GetTagTransform(sideLength, buff.markerCorners[i], this);
		buff.markerViews[i] = CameraView(CameraIdx, buff.markerIDs[i], TagTransform);
	}
	buff.HasViews = true;
}

bool Camera::GetMarkerData(int BufferIndex, vector<int>& markerIDs, vector<vector<Point2f>>& markerCorners)
{
	if (!FrameBuffer[BufferIndex].HasAruco)
	{
		return false;
	}
	markerIDs = FrameBuffer[BufferIndex].markerIDs;
	markerCorners = FrameBuffer[BufferIndex].markerCorners;
	return true;
}

int Camera::GetCameraViewsSize(int BufferIndex)
{
	return FrameBuffer[BufferIndex].markerViews.size();
}

bool Camera::GetCameraViews(int BufferIndex, vector<CameraView>& views)
{
	if (!FrameBuffer[BufferIndex].HasViews)
	{
		return false;
	}
	views = FrameBuffer[BufferIndex].markerViews;
	return true;
}

vector<Camera*> autoDetectCameras(CameraStartType Start, String Filter, String CalibrationFile, bool silent)
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

    vector<Camera*> detected;
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
		if (device.device_description.find(Filter) != String::npos)
		{
			CameraSettings settings;
			settings.Resolution = GetFrameSize();
			settings.Framerate = GetCaptureFramerate();
			settings.FramerateDivider = 1;
			settings.DeviceInfo = device;
			settings.BufferSize = 2;
			settings.StartType = Start;

			settings.StartPath = "";
			settings.ApiID = -1;

			Camera* cam = new Camera(settings);
			readCameraParameters(String("../calibration/") + CalibrationFile, cam->CameraMatrix, cam->distanceCoeffs, settings.Resolution);
			//cout << "Camera matrix : " << cam->CameraMatrix << " / Distance coeffs : " << cam->distanceCoeffs << endl;
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
			cerr << "ERROR! Unable to open camera " << Cameras[i]->GetCameraSettings().DeviceInfo.device_description;
		}
	}
	return true;
}