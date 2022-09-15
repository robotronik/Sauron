#include "Calibrate.hpp"

#include <iostream> // for standard I/O
#include <string>   // for strings
#include <sstream>  // string to number conversion

#include <filesystem>

#include <opencv2/core.hpp>     // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>  // OpenCV window I/O
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <opencv2/cudawarping.hpp>

#include "thirdparty/serialib.h"
#include "GlobalConf.hpp"
#include "Cameras/Camera.hpp"
#include "Cameras/VideoCaptureCamera.hpp"
#include "data/CameraView.hpp"
#include "data/Calibfile.hpp"
#include "data/FrameCounter.hpp"

using namespace std;
using namespace cv;
namespace fs = std::filesystem;

const float CalibrationSquareEdge = 0.04; //m
const Size CheckerSize = Size(6, 4);
const String TempImgPath = "TempCalib";
const String CalibWindowName = "Calibration";


void CreateKnownBoardPos(Size BoardSize, float squareEdgeLength, vector<Point3f>& corners)
{
	for (int i = 0; i < BoardSize.height; i++)
	{
		for (int j = 0; j < BoardSize.width; j++)
		{
			corners.push_back(Point3f(j * squareEdgeLength, i * squareEdgeLength, 0));
		}   
	}
}

void GetChessboardCorners(vector<UMat> images, vector<vector<Point2f>>& corners, bool showResults = false)
{
	for (int i = 0; i < images.size(); i++)
	{
		vector<Point2f> cornersBuffer;
		bool found = findChessboardCorners(images[i], CheckerSize, cornersBuffer, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
		if (found)
		{
			corners.push_back(cornersBuffer);
		}

		if (showResults)
		{
			drawChessboardCorners(images[i], CheckerSize, cornersBuffer, found);
			imshow("Looking for corners", images[i]);
			waitKey(0);
		}
	}
	
}

void CameraCalibration(vector<vector<Point2f>> CheckerboardImageSpacePoints, Size BoardSize, Size resolution, float SquareEdgeLength, Mat& CameraMatrix, Mat& DistanceCoefficients, Camera* CamToCalibrate)
{
	vector<vector<Point3f>> WorldSpaceCornerPoints(1);
	CreateKnownBoardPos(BoardSize, SquareEdgeLength, WorldSpaceCornerPoints[0]);
	WorldSpaceCornerPoints.resize(CheckerboardImageSpacePoints.size(), WorldSpaceCornerPoints[0]);

	Size FrameSize = resolution;
	vector<Mat> rVectors, tVectors;
	CameraMatrix = Mat::eye(3, 3, CV_64F);
	DistanceCoefficients = Mat::zeros(Size(4,1), CV_64F);
	CameraMatrix = initCameraMatrix2D(WorldSpaceCornerPoints, CheckerboardImageSpacePoints, FrameSize);
	cout << "Camera matrix at start : " << CameraMatrix << endl;
	UMat undistorted;
	CamToCalibrate->Calibrate(WorldSpaceCornerPoints, CheckerboardImageSpacePoints, FrameSize, CameraMatrix, DistanceCoefficients, 
		rVectors, tVectors);
}

vector<String> CalibrationImages()
{
	vector<String> pathos;
	for (const auto & entry : fs::directory_iterator(TempImgPath))
    {
		pathos.push_back(entry.path().string());
		//cout << entry.path().string() << endl;
	}
	return pathos;
}

int LastIdx(vector<String> Pathes)
{
	int next = -1;
	for (int i = 0; i < Pathes.size(); i++)
	{
		String stripped = Pathes[i].substr(TempImgPath.length()+1, Pathes[i].length() - TempImgPath.length()-1 - String(".png").length());
		int thatidx = stoi(stripped);
		next = next < thatidx ? thatidx : next;
	}
	return next;
}

Size ReadAndCalibrate(Mat& CameraMatrix, Mat& DistanceCoefficients, Camera* CamToCalibrate)
{
	vector<String> pathes = CalibrationImages();
	size_t numpathes = pathes.size();
	vector<vector<Point2f>> savedPoints;
	savedPoints.resize(numpathes);
	vector<Size> resolutions;
	resolutions.resize(numpathes);
	vector<bool> valids;
	valids.resize(numpathes);
	parallel_for_(Range(0, numpathes), [&](const Range InRange)
	{
		for (size_t i = InRange.start; i < InRange.end; i++)
		{
			Mat frame = imread(pathes[i], IMREAD_GRAYSCALE);
			resolutions[i] = frame.size();
			vector<Point2f> foundPoints;
			bool found = findChessboardCorners(frame, CheckerSize, foundPoints, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
			if (found)
			{
				TermCriteria criteria(TermCriteria::COUNT | TermCriteria::EPS, 100, 0.001);
				cornerSubPix(frame, foundPoints, Size(4,4), Size(-1,-1), criteria);
				//Scalar sharpness = estimateChessboardSharpness(frame, CheckerSize, foundPoints);
				savedPoints[i] = foundPoints;
				valids[i] = true;
			}
			else
			{
				valids[i] =false;
				cout << "Failed to find chessboard in image " << pathes[i] << " index " << i << endl;
			}
			
		}
	});

	cout << "Images are done loading, starting calibration..." <<endl;
	for (int i = numpathes - 1; i >= 0; i--)
	{
		if (!valids[i])
		{
			resolutions.erase(resolutions.begin() + i);
			savedPoints.erase(savedPoints.begin() + i);
			valids.erase(valids.begin() + i);
			pathes.erase(pathes.begin() + i);
		}
	}
	
	bool multires = false;
	vector<Size> sizes;
	for (size_t i = 0; i < numpathes; i++)
	{
		bool hasres = false;
		for (size_t j = 0; j < sizes.size(); j++)
		{
			if (sizes[j] == resolutions[i])
			{
				hasres = true;
				break;
			}
		}
		if (!hasres)
		{
			sizes.push_back(resolutions[i]);
		}
	}
	
	if (sizes.size() == 1)
	{
		UMat image = imread(pathes[0], IMREAD_COLOR).getUMat(AccessFlag::ACCESS_READ);
		CameraCalibration(savedPoints, CheckerSize, sizes[0], CalibrationSquareEdge, CameraMatrix, DistanceCoefficients, CamToCalibrate);
		cout << "Calibration done ! Matrix : " << CameraMatrix << " / Distance Coefficients : " << DistanceCoefficients << endl;
		return sizes[0];
	}
	else
	{
		cerr << "ERROR : " << sizes.size() << " different resolutions were used in the calibration. That's fixable but fuck you." << endl;
		cerr << "-Cordialement, le trez 2021/2022 Robotronik (Gabriel Zerbib)" << endl;
		for (size_t i = 0; i < sizes.size(); i++)
		{
			cerr << "@" << sizes[i] <<endl;
			for (size_t j = 0; j < numpathes; j++)
			{
				cerr << " -" << pathes[j] <<endl;
			}
		}
		return Size(0,0);
	}
}

bool docalibration(CameraSettings CamSett)
{
	bool HasCamera = CamSett.IsValid();

	Camera* CamToCalib = new VideoCaptureCamera(CamSett);
	if (HasCamera)
	{
		CamToCalib->StartFeed();
	}
	
	

	Mat CameraMatrix;
	Mat distanceCoefficients;

	bool ShowUndistorted = false;
	bool AutoCapture = false;
	float AutoCaptureFramerate = 2;
	double AutoCaptureStart;
	int LastAutoCapture;

	
	fs::create_directory(TempImgPath);

	if (!HasCamera)
	{
		cout << "No camera was found, calibrating from saved images" << endl;
		Size framesize = ReadAndCalibrate(CameraMatrix, distanceCoefficients, CamToCalib);
		writeCameraParameters("NoCam", CameraMatrix, distanceCoefficients, framesize);
		vector<String> pathes = CalibrationImages();
		for (int i = 0; i < pathes.size(); i++)
		{

		}
		
		return true;
	}
	CamToCalib->StartFeed();

	namedWindow(CalibWindowName, WINDOW_NORMAL);
	setWindowProperty(CalibWindowName, WND_PROP_FULLSCREEN, WINDOW_FULLSCREEN);
	
	vector<String> pathes = CalibrationImages();
	int nextIdx = LastIdx(pathes) +1;

	FrameCounter fps;
	int failed = 0;
	while (true)
	{
		UMat frame, frameresized;
		bool CaptureImageThisFrame = false;
		cuda::GpuMat gpuframe, gpuresized;
		cuda::Stream resizestream;
		
		if (!CamToCalib->Read(0))
		{
			//cout<< "read fail" <<endl;
			failed++;
			if (failed >10)
			{
				break;
			}
			
			continue;
		}
		CamToCalib->GetFrame(0, frame);
		if (GetScreenSize() != CamSett.Resolution)
		{
			gpuframe.upload(frame, resizestream);
			cuda::resize(gpuframe, gpuresized, GetScreenSize(), 0, 0, 1, resizestream);
			gpuresized.download(frameresized);
		}
		else
		{
			frameresized = frame;
		}
		
		
		

		//cout << "read success" << endl;
		//drawChessboardCorners(drawToFrame, CheckerSize, foundPoints, found);
		char character = waitKey(1);

		if (ShowUndistorted)
		{
			UMat frameundist;
			CamToCalib->Undistort(0);

			CamToCalib->GetOutputFrame(0, frame, CamSett.Resolution);
		}

		switch (character)
		{
		case 'a':
			if (!ShowUndistorted)
			{
				AutoCapture = !AutoCapture;
				AutoCaptureStart = fps.GetAbsoluteTime();
				LastAutoCapture = 0;
			}
			break;
		case ' ':
			//save image
			if (!ShowUndistorted)
			{
				CaptureImageThisFrame = true;
			}
			break;
		
		case 13: //enter
			//start calib
			if (ShowUndistorted)
			{
				ShowUndistorted = false;
			}
			else
			{
				ReadAndCalibrate(CameraMatrix, distanceCoefficients, CamToCalib);
				CamToCalib->SetCalibrationSetting(CameraMatrix, distanceCoefficients);
				double apertureWidth = 4.96, apertureHeight = 3.72, fovx, fovy, focalLength, aspectRatio;
				Point2d principalPoint;
				calibrationMatrixValues(CameraMatrix, CamToCalib->GetCameraSettings().Resolution, apertureWidth, apertureHeight, fovx, fovy, focalLength, principalPoint, aspectRatio);
				cout << "Computed camera parameters for sensor of size " << apertureWidth << "x" << apertureHeight <<"mm :" << endl
				<< " fov:" << fovx << "x" << fovy << "°, focal length=" << focalLength << ", aspect ratio=" << aspectRatio << endl
				<< "Principal point @ " << principalPoint << endl;
				writeCameraParameters(CamToCalib->GetCameraSettings().DeviceInfo.device_description, CameraMatrix, distanceCoefficients, CamToCalib->GetCameraSettings().Resolution);
				//distanceCoefficients = Mat::zeros(8, 1, CV_64F);
				ShowUndistorted = true;
			}
			break;

		case 27:
			//exit
			return true;
			break;

		default:
			break;
		}

		if (AutoCapture)
		{
			int autoCaptureIdx = floor((fps.GetAbsoluteTime() - AutoCaptureStart)*AutoCaptureFramerate);
			if (autoCaptureIdx > LastAutoCapture)
			{
				LastAutoCapture++;
				CaptureImageThisFrame = true;
			}
			
		}
		

		if (CaptureImageThisFrame)
		{
			vector<Point2f> foundPoints;
			UMat grayscale;
			cvtColor(frame, grayscale, COLOR_BGR2GRAY);
			bool found = checkChessboard(grayscale, CheckerSize);
			imwrite(TempImgPath + "/" + to_string(nextIdx++) + ".png", frame);
			putText(frame, "Image captured !", Point(100,100), FONT_HERSHEY_SIMPLEX, 2, Scalar(255,0,0), 4);
		}
		resizestream.waitForCompletion();
		
		fps.AddFpsToImage(frameresized, fps.GetDeltaTime());
		imshow(CalibWindowName, frameresized);
	}
	return true;
}
