#include "Cameras/Camera.hpp"

#include <iostream> // for standard I/O
#include <math.h>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

#ifdef WITH_CUDA
#include <opencv2/cudacodec.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>
#else
#include <opencv2/imgproc.hpp>
#endif

#include "math3d.hpp"
#include "data/Calibfile.hpp"
#include "data/CameraView.hpp"

#include "ObjectTracker.hpp"
#include "GlobalConf.hpp"
#include "data/FVector2D.hpp"

using namespace cv;
using namespace std;

void Camera::RegisterError()
{
	errors += 10;
}

void Camera::RegisterNoError()
{
	errors = std::max(0, errors -1);
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
	HasUndistortionMaps = false;
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
		Size cammatsz = setcopy.CameraMatrix.size();
		if (cammatsz.height != 3 || cammatsz.width != 3)
		{
			RegisterError();
			cerr << "Asking for undistortion but camera matrix is invalid ! Camera " << setcopy.DeviceInfo.device_description << endl;
			return;
		}
		
		
		//cout << "Creating undistort map using Camera Matrix " << endl << setcopy.CameraMatrix << endl 
		//<< " and Distance coeffs " << endl << setcopy.distanceCoeffs << endl;
		Mat map1, map2, newCamMat;
		float balance = 0.8;
		//fisheye::estimateNewCameraMatrixForUndistortRectify(setcopy.CameraMatrix, setcopy.distanceCoeffs, setcopy.Resolution, Mat::eye(3, 3, CV_32F), newCamMat, balance);
		initUndistortRectifyMap(setcopy.CameraMatrix, setcopy.distanceCoeffs, Mat::eye(3,3, CV_64F), 
		setcopy.CameraMatrix, setcopy.Resolution, CV_32FC1, map1, map2);
		#ifdef WITH_CUDA
		UndistMap1.upload(map1);
		UndistMap2.upload(map2);
		#else
		map1.copyTo(UndistMap1);
		map2.copyTo(UndistMap2);
		#endif
		HasUndistortionMaps = true;
	}

	BufferedFrame& buff = FrameBuffer[BufferIdx];
	
	#ifdef WITH_CUDA
	if (!buff.FrameRaw.MakeGPUAvailable())
	{
		return;
	}
	
	buff.FrameUndistorted.GPUFrame = cuda::createContinuous(buff.FrameRaw.GPUFrame.size(), buff.FrameRaw.GPUFrame.type());
	//cout << "CV_32F=" << CV_32F << " 1=" << UndistMap1.type() << " 2=" << UndistMap2.type() << endl;
	cuda::remap(buff.FrameRaw.GPUFrame, buff.FrameUndistorted.GPUFrame, UndistMap1, UndistMap2, INTER_LINEAR);
	buff.FrameUndistorted.HasGPU = true;
	buff.FrameUndistorted.HasCPU = false;
	#else
	if (!buff.FrameRaw.MakeCPUAvailable())
	{
		return;
	}
	remap(buff.FrameRaw.CPUFrame, buff.FrameUndistorted.CPUFrame, UndistMap1, UndistMap2, INTER_LINEAR);
	buff.FrameUndistorted.HasCPU = true;
	#endif
}

void Camera::GetFrameUndistorted(int BufferIndex, UMat& frame)
{
	BufferedFrame& buff = FrameBuffer[BufferIndex];
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

void Camera::GetOutputFrame(int BufferIndex, UMat& OutFrame, Rect window)
{
	BufferedFrame& buff = FrameBuffer[BufferIndex];
	MixedFrame& frametoUse = buff.FrameUndistorted;
	if (!frametoUse.IsValid())
	{
		return;
	}
	double fz = 1;
	Size basesize = frametoUse.GetSize();
	Size winsize = window.size();
	Point2f offset(window.x, window.y);
	if (winsize == basesize)
	{
		frametoUse.MakeCPUAvailable();
		if (frametoUse.CPUFrame.channels() != 3)
		{
			cvtColor(frametoUse.CPUFrame, OutFrame(window), COLOR_GRAY2BGR);
		}
		else
		{
			frametoUse.CPUFrame.copyTo(OutFrame(window));
		}
		
	}
	else
	{
		Rect roi = ScaleToFit(basesize, window);
		fz = (double)roi.width / basesize.width;
		offset = roi.tl();
		#ifdef WITH_CUDA
		if (frametoUse.HasGPU)
		{
			cuda::GpuMat resz;
			UMat dl;
			cuda::resize(frametoUse.GPUFrame, resz, roi.size(), fz, fz, INTER_AREA);
			resz.download(OutFrame(roi));
		}
		#else
		if(0){}
		#endif
		else
		{
			resize(frametoUse.CPUFrame, OutFrame(roi), roi.size(), fz, fz, INTER_AREA);
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
				marker.push_back(FrameBuffer[BufferIndex].markerCorners[i][j]*fz + offset);
			}
			raruco.push_back(marker);
		}
		aruco::drawDetectedMarkers(OutFrame, raruco, FrameBuffer[BufferIndex].markerIDs);
	}
}

vector<ObjectData> Camera::ToObjectData(int BaseNumeral)
{
	ObjectData robot;
	robot.identity.numeral = BaseNumeral;
	robot.identity.type = PacketType::Camera;
	robot.location = Location;
	return {robot};
}

Affine3d Camera::GetObjectTransform(CameraArucoData& CameraData, float& Surface)
{
	Surface = -1;
	return Affine3d::Identity();
}

///////////////////////////////
// SECTION ArucoCamera
///////////////////////////////

void ArucoCamera::RescaleFrames(int BufferIdx)
{
	BufferedFrame& buff = FrameBuffer[BufferIdx];
	#ifdef WITH_CUDA
	if (!buff.FrameUndistorted.MakeGPUAvailable())
	{
		return;
	}
	#else
	if (!buff.FrameUndistorted.MakeCPUAvailable())
	{
		return;
	}
	#endif
	Size rescale = GetArucoReduction();

	#ifdef WITH_CUDA
	cuda::cvtColor(buff.FrameUndistorted.GPUFrame, buff.GrayFrame.GPUFrame, buff.FrameUndistorted.GPUFrame.channels() == 3 ? COLOR_BGR2GRAY : COLOR_BGRA2GRAY);
	if (rescale == buff.FrameUndistorted.GPUFrame.size())
	{
		buff.RescaledFrame = MixedFrame(buff.GrayFrame.GPUFrame);
	}
	else
	{
		cuda::resize(buff.GrayFrame.GPUFrame, buff.RescaledFrame.GPUFrame, rescale, 0, 0, INTER_AREA);
	}
	buff.GrayFrame.HasGPU = true;
	buff.GrayFrame.HasCPU = false;
	buff.RescaledFrame.HasGPU = true;
	buff.RescaledFrame.HasCPU = false;
	#else
	bool rgb = buff.FrameUndistorted.CPUFrame.channels() == 3;
	cvtColor(buff.FrameUndistorted.CPUFrame, buff.GrayFrame.CPUFrame, rgb ? COLOR_BGR2GRAY : COLOR_BGRA2GRAY);
	
	if (rescale == buff.FrameUndistorted.CPUFrame.size())
	{
		buff.RescaledFrame = MixedFrame(buff.GrayFrame.CPUFrame);
	}
	else
	{
		resize(buff.GrayFrame.CPUFrame, buff.RescaledFrame.CPUFrame, rescale, 0, 0, INTER_AREA);
	}
	buff.GrayFrame.HasCPU = true;
	buff.RescaledFrame.HasCPU = true;
	#endif
}

void ArucoCamera::detectMarkers(int BufferIndex, Ptr<aruco::Dictionary> dict, Ptr<aruco::DetectorParameters> params)
{
	BufferedFrame& buff = FrameBuffer[BufferIndex];

	Size framesize = GetFrameSize();
	Size rescaled = GetArucoReduction();

	if(!buff.GrayFrame.MakeCPUAvailable())
	{
		return;
	}
	if(!buff.RescaledFrame.MakeCPUAvailable())
	{
		return;
	}
	

	vector<vector<Point2f>> &corners = buff.markerCorners;
	corners.clear();
	vector<int> &IDs = buff.markerIDs;
	IDs.clear();
	aruco::detectMarkers(buff.RescaledFrame.CPUFrame, dict, corners, IDs, params);

	FVector2D scalefactor = FVector2D(framesize)/FVector2D(rescaled);

	for (int j = 0; j < corners.size(); j++)
	{
		for (size_t k = 0; k < 4; k++)
		{
			FVector2D pos = FVector2D(corners[j][k]) * scalefactor;
			corners[j][k] = pos;
		}
	}

	float reductionFactors = GetReductionFactor();

	if (rescaled == Settings.Resolution) //fast path for running at native res
	{
	}
	else
	{
		UMat &framegray = buff.GrayFrame.CPUFrame;

		for (int ArucoIdx = 0; ArucoIdx < IDs.size(); ArucoIdx++)
		{
			Size window = Size(reductionFactors, reductionFactors);
			cornerSubPix(framegray, corners[ArucoIdx], window, Size(-1,-1), TermCriteria(TermCriteria::COUNT | TermCriteria::EPS, 100, 0.01));
		}
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