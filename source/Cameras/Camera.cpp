#include "Cameras/Camera.hpp"

#include <iostream> // for standard I/O
#include <iomanip>  // for controlling float print precision
#include <sstream>  // string to number conversion

#include <opencv2/cudacodec.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>

#include <opencv2/calib3d.hpp>

#include "thirdparty/list-devices.hpp"
#include "thirdparty/serialib.h"

#include "data/Calibfile.hpp"
#include "data/CameraView.hpp"

#include "TrackedObjects/TrackedObject.hpp" //CameraView
#include "ObjectTracker.hpp"
#include "GlobalConf.hpp"
#include "data/FVector2D.hpp"

vector<CameraSettings> Camera::autoDetectCameras(CameraStartType Start, String Filter, String CalibrationFile, bool silent)
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
	bool invertedFilter = false;
	if (Filter.length() > 1 && Filter[0] == '!')
	{
		invertedFilter = true;
		Filter = Filter.substr(1);
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
		if ((device.device_description.find(Filter) != String::npos) ^ invertedFilter)
		{
			CameraSettings settings;
			settings.Resolution = GetFrameSize();
			settings.Framerate = GetCaptureFramerate();
			settings.FramerateDivider = 1;
			settings.DeviceInfo = device;
			settings.BufferSize = 2;
			settings.StartType = Start;

			//these only get populated when StartFeed is called
			settings.StartPath = "";
			settings.ApiID = -1;

			String CalibrationRoot = String("../calibration/");
			String CalibrationPath = CalibrationRoot + settings.DeviceInfo.device_description;
			if (CalibrationFile.length() > 0)
			{
				CalibrationPath = CalibrationRoot + CalibrationFile;
			}

			readCameraParameters(CalibrationPath, settings.CameraMatrix, settings.distanceCoeffs, settings.Resolution);
			//cout << "Camera matrix : " << cam->CameraMatrix << " / Distance coeffs : " << cam->distanceCoeffs << endl;
			detected.push_back(settings);
		}
	}
    return detected;
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
	Settings = InSettings;
	return true;
}

bool Camera::SetCalibrationSetting(Mat CameraMatrix, Mat DistanceCoefficients)
{
	Settings.CameraMatrix = CameraMatrix;
	Settings.distanceCoeffs = DistanceCoefficients;
	return true;
}

void Camera::GetCameraSettingsAfterUndistortion(Mat& CameraMatrix, Mat& DistanceCoefficients)
{
	CameraMatrix = Settings.CameraMatrix;
	DistanceCoefficients = Mat::zeros(4,1, CV_64F);
}

BufferStatus Camera::GetStatus(int BufferIndex)
{
	return FrameBuffer[BufferIndex].Status;
}

bool Camera::StartFeed()
{
	cerr << "ERROR : Tried to start feed on base class Camera !" << endl;
	return false;
}

bool Camera::Grab(int BufferIndex)
{
	cerr << "ERROR : Tried to grab on base class Camera !" << endl;
	return false;
}

bool Camera::Read(int BufferIndex)
{
	cerr << "ERROR : Tried to read on base class Camera !" << endl;
	return false;
}

bool Camera::InjectImage(int BufferIndex, UMat& frame)
{
	cerr << "ERROR : Tried to read on base class Camera !" << endl;
	return false;
}

void Camera::Undistort(int BufferIdx)
{
	CameraSettings setcopy = GetCameraSettings();
	if (!HasUndistortionMaps)
	{
		cout << "Creating undistort map using Camera Matrix " << endl << setcopy.CameraMatrix << endl 
		<< " and Distance coeffs " << endl << setcopy.distanceCoeffs << endl;
		Mat map1, map2;
		initUndistortRectifyMap(setcopy.CameraMatrix, setcopy.distanceCoeffs, Mat::eye(3,3, CV_64F), 
		setcopy.CameraMatrix, setcopy.Resolution, CV_32FC1, map1, map2);
		UndistMap1.upload(map1);
		UndistMap2.upload(map2);
		HasUndistortionMaps = true;
	}

	BufferedFrame& buff = FrameBuffer[BufferIdx];
	
	if (!buff.Status.HasCaptured)
	{
		return;
	}
	
	if (!buff.FrameRaw.MakeGPUAvailable())
	{
		return;
	}
	
	buff.FrameUndistorted.GPUFrame = cuda::createContinuous(buff.FrameRaw.GPUFrame.size(), buff.FrameRaw.GPUFrame.type());
	//cout << "CV_32F=" << CV_32F << " 1=" << UndistMap1.type() << " 2=" << UndistMap2.type() << endl;
	cuda::remap(buff.FrameRaw.GPUFrame, buff.FrameUndistorted.GPUFrame, UndistMap1, UndistMap2, INTER_LINEAR);
	buff.FrameUndistorted.HasGPU = true;
	buff.FrameUndistorted.HasCPU = false;
	buff.Status.HasUndistorted = true;
}

void Camera::GetFrameUndistorted(int BufferIndex, UMat& frame)
{
	BufferedFrame& buff = FrameBuffer[BufferIndex];
	if (!buff.Status.HasUndistorted)
	{
		return;
	}
	if (!buff.FrameUndistorted.MakeCPUAvailable())
	{
		return;
	}
	frame = buff.FrameUndistorted.CPUFrame;
}

void Camera::Calibrate(vector<vector<Point3f>> objectPoints,
	vector<vector<Point2f>> imagePoints, Size imageSize,
	Mat& cameraMatrix, Mat& distCoeffs,
	OutputArrayOfArrays rvecs, OutputArrayOfArrays tvecs)
{
	calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, 
	CALIB_RATIONAL_MODEL, TermCriteria(TermCriteria::COUNT, 50, DBL_EPSILON));
}

void Camera::GetFrame(int BufferIndex, UMat& OutFrame)
{
	FrameBuffer[BufferIndex].FrameRaw.GetCPUFrame(OutFrame);
}

void Camera::GetOutputFrame(int BufferIndex, UMat& OutFrame, Size winsize)
{
	BufferedFrame& buff = FrameBuffer[BufferIndex];
	if (!buff.FrameUndistorted.IsValid())
	{
		return;
	}
	double fz = 1;
	Size basesize = buff.FrameUndistorted.GetSize();
	if (winsize == basesize)
	{
		buff.FrameUndistorted.MakeCPUAvailable();
		buff.FrameUndistorted.CPUFrame.copyTo(OutFrame);
	}
	else
	{
		double fx = (double)winsize.width / basesize.width;
		double fy = (double)winsize.height / basesize.height;
		fz = min(fx, fy);
		if (buff.FrameUndistorted.HasGPU)
		{
			cuda::GpuMat resz;
			cuda::resize(buff.FrameUndistorted.GPUFrame, resz, Size(), fz, fz, INTER_AREA);
			resz.download(OutFrame);
		}
		else
		{
			resize(buff.FrameUndistorted.CPUFrame, OutFrame, Size(), fz, fz, INTER_AREA);
		}
	}
	if (FrameBuffer[BufferIndex].Status.HasAruco)
	{
		vector<vector<Point2f>> raruco;
		for (int i = 0; i < FrameBuffer[BufferIndex].markerCorners.size(); i++)
		{
			vector<Point2f> marker;
			for (int j = 0; j < FrameBuffer[BufferIndex].markerCorners[i].size(); j++)
			{
				marker.push_back(FrameBuffer[BufferIndex].markerCorners[i][j]*fz);
			}
			raruco.push_back(marker);
		}
		aruco::drawDetectedMarkers(OutFrame, raruco, FrameBuffer[BufferIndex].markerIDs);
	}
}

///////////////////////////////
// SECTION ArucoCamera
///////////////////////////////

void ArucoCamera::RescaleFrames(int BufferIdx)
{
	BufferedFrame& buff = FrameBuffer[BufferIdx];
	if (!buff.Status.HasUndistorted)
	{
		return;
	}
	
	if (!buff.FrameUndistorted.MakeGPUAvailable())
	{
		return;
	}

	vector<Size> rescales = GetArucoReductions();
	buff.rescaledFrames.resize(rescales.size());

	for (int i = 0; i < rescales.size(); i++)
	{
		cuda::GpuMat rescaled;
		cuda::resize(buff.FrameUndistorted.GPUFrame, rescaled, rescales[i], 0, 0, INTER_AREA);
		cuda::cvtColor(rescaled, buff.rescaledFrames[i].GPUFrame, COLOR_BGR2GRAY);
		buff.rescaledFrames[i].HasGPU = true;
		buff.rescaledFrames[i].HasCPU = false;
	}
	buff.Status.HasResized = true;
}

void ArucoCamera::detectMarkers(int BufferIndex, Ptr<aruco::Dictionary> dict, Ptr<aruco::DetectorParameters> params)
{
	/*cuda::GpuMat framegpu; framegpu.upload(frame);
	cuda::cvtColor(framegpu, framegpu, COLOR_BGR2GRAY);
	cuda::threshold(framegpu, framegpu, 127, 255, THRESH_BINARY);
	UMat framethreshed; framegpu.download(framethreshed);*/

	BufferedFrame& buff = FrameBuffer[BufferIndex];

	//the first vectors is for each resolution
	//the second vector is for each tag
	//all the positions are in pixels of the base frame

	//corners of the tags, in pixel position
	vector<vector<vector<Point2f>>> AllCorners;
	//ids of the tags
	vector<vector<int>> AllIDs;
	//centers of the tags, in pixel position
	vector<vector<FVector2D<double>>> Centers;
	int nbframes = buff.rescaledFrames.size();
	AllCorners.resize(nbframes);
	AllIDs.resize(nbframes);
	Centers.resize(nbframes);
	Size framesize = GetFrameSize();
	vector<Size> rescaled = GetArucoReductions();


	
	

	//Range InRange(0, nbframes);
	parallel_for_(Range(0,nbframes), [&](const Range InRange)
	{
		for (size_t i = InRange.start; i < InRange.end; i++)
		{
			vector<vector<Point2f>> corners;
			vector<int>& IDs = AllIDs[i];
			MixedFrame frameM;
			UMat frame;
			if (!buff.GetRescaledFrame(i, frameM))
			{
				continue;
			}
			frameM.GetCPUFrame(frame);
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

	if (rescaled[0] == Settings.Resolution && nbframes == 1) //fast path for running at native res
	{
		buff.markerCorners = AllCorners[0];
		buff.markerIDs = AllIDs[0];
	}
	else
	{
		UMat framebase, framegray;
		if (!buff.FrameUndistorted.GetCPUFrame(framebase))
		{
			return;
		}
		cvtColor(framebase, framegray, COLOR_BGR2GRAY);

		//Lower resolution processing to increase accuracy while keeping speed
		for (int Lower = 0; Lower < nbframes; Lower++)
		{
			for (int ArucoLower = 0; ArucoLower < AllIDs[Lower].size(); ArucoLower++)
			{
				markerIDs.push_back(AllIDs[Lower][ArucoLower]);
				vector<Point2f> cornersTemp = AllCorners[Lower][ArucoLower];
				Size window = Size(reductionFactors[Lower], reductionFactors[Lower]);
				cornerSubPix(framegray, cornersTemp, window, Size(-1,-1), TermCriteria(TermCriteria::COUNT | TermCriteria::EPS, 100, 0.01));
				corners.push_back(cornersTemp);
				//cout << "Found aruco " << AllIDs[Lower][ArucoLower] << " at height " << Lower << endl;

				//If there are lower-resolution appearances of this tag, remove them as they'll have a lower accuracy
				for (int Higher = Lower +1; Higher < nbframes; Higher++)
				{
					for (int ArucoHigher = 0; ArucoHigher < AllIDs[Higher].size(); ArucoHigher++)
					{
						//check same id
						if (AllIDs[Lower][ArucoLower] == AllIDs[Higher][ArucoHigher])
						{
							//check they are not too far apart on the frame
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
	}
	
	
	buff.Status.HasAruco = true;
}

bool ArucoCamera::GetMarkerData(int BufferIndex, CameraArucoData& CameraData)
{
	if (!FrameBuffer[BufferIndex].Status.HasAruco)
	{
		return false;
	}
	CameraData.TagIDs = FrameBuffer[BufferIndex].markerIDs;
	CameraData.TagCorners = FrameBuffer[BufferIndex].markerCorners;
	GetCameraSettingsAfterUndistortion(CameraData.CameraMatrix, CameraData.DistanceCoefficients);
	CameraData.CameraTransform = Location;
	return true;
}

void ArucoCamera::SolveMarkers(int BufferIndex, int CameraIdx, ObjectTracker* registry)
{
	BufferedFrame& buff = FrameBuffer[BufferIndex];
	if (!buff.Status.HasAruco)
	{
		return;
	}
	buff.markerViews.resize(buff.markerIDs.size());
	Mat CamMatrix, distCoefs;
	GetCameraSettingsAfterUndistortion(CamMatrix, distCoefs);
	for (int i = 0; i < buff.markerIDs.size(); i++)
	{
		float sideLength = registry->GetArucoSize(buff.markerIDs[i]);
		Affine3d TagTransform = GetTagTransform(sideLength, buff.markerCorners[i], CamMatrix, distCoefs);
		buff.markerViews[i] = CameraView(CameraIdx, buff.markerIDs[i], TagTransform);
	}
	buff.Status.HasViews = true;
}

int ArucoCamera::GetCameraViewsSize(int BufferIndex)
{
	return FrameBuffer[BufferIndex].markerViews.size();
}

bool ArucoCamera::GetCameraViews(int BufferIndex, vector<CameraView>& views)
{
	if (!FrameBuffer[BufferIndex].Status.HasViews)
	{
		return false;
	}
	views = FrameBuffer[BufferIndex].markerViews;
	return true;
}