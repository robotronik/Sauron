/* 

	Ce fichier est un script conçu pour calibrer une caméra en utilisant une mire d'étalonnage comme un échiquier. Basé sur l'API OpenCV et utilise des bibliothèque d'accélération matérielle CUDA et OpenCL pour accélérer le traitement d'image ainsi que la prise en charge du multi-threading pour accélérer le traitement des données.

	Dans l'ensemble, le script capture des images de la mire d'étalonnage à l'aide de la caméra, localise les coins sur l'échiquier et utilise ces informations pour calculer les paramètres intrinsèques de la caméra pour la calibration.

*/

#include "Calibrate.hpp"

#include <iostream> // for standard I/O
#include <string>   // for strings
#include <sstream>  // string to number conversion

#include <filesystem>
#include <thread>
#include <mutex>

#include <opencv2/core.hpp>		// Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>  // OpenCV window I/O
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#ifdef WITH_CUDA
#include <opencv2/cudawarping.hpp>
#endif

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

const String TempImgPath = "TempCalib";
const String CalibWindowName = "Calibration";


// Cette fonction crée un ensemble de points 3D correspondant aux coins de l'échiquier
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

// Cette fonction effectue l'étalonnage de la caméra. Elle prend comme entrée les points de l'échiquier dans l'image et les correspondances de ces points dans l'espace 3D. Elle renvoie la matrice de la caméra et les coefficients de distorsion.
void CameraCalibration(vector<vector<Point2f>> CheckerboardImageSpacePoints, vector<string> ImagePaths, Size BoardSize, Size resolution, float SquareEdgeLength, Mat& CameraMatrix, Mat& DistanceCoefficients, Camera* CamToCalibrate)
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
	CamToCalibrate->Calibrate(WorldSpaceCornerPoints, CheckerboardImageSpacePoints, ImagePaths, FrameSize, CameraMatrix, DistanceCoefficients, 
		rVectors, tVectors);
}

// Cette fonction récupère la liste des chemins des images d'étalonnage stockées dans le dossier TempImgPath.
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
		try
		{
			int thatidx = stoi(stripped);
			next = next < thatidx ? thatidx : next;
		}
		catch(const std::exception& e)
		{
			cout << "Failed to stoi " << stripped <<endl;
			//std::cerr << e.what() << '\n';
		}
	}
	return next;
}

// Cette fonction lit les images de la mire d'étalonnage et effectue l'étalonnage de la caméra dans un thread séparé.
Size ReadAndCalibrate(Mat& CameraMatrix, Mat& DistanceCoefficients, Camera* CamToCalibrate)
{
	auto calconf = GetCalibrationConfig();
	Size CheckerSize = calconf.NumIntersections;
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
		CameraCalibration(savedPoints, pathes, CheckerSize, sizes[0], calconf.SquareSideLength/1000.f, CameraMatrix, DistanceCoefficients, CamToCalibrate);
		cout << "Calibration done ! Matrix : " << CameraMatrix << " / Distance Coefficients : " << DistanceCoefficients << endl;
		return sizes[0];
	}
	else if (sizes.size() == 0)
	{
		cout << "Il faut prendre des images pour pouvoir calibrer la caméra quand même..." << endl;
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
	}
	return Size(0,0);
}

Camera* CamToCalib;
Mat CameraMatrix;
Mat distanceCoefficients;

thread *CalibrationThread;
bool Calibrating = false;
bool ShowUndistorted = false;

void CalibrationWorker()
{
	Calibrating = true;
	Size resolution = ReadAndCalibrate(CameraMatrix, distanceCoefficients, CamToCalib);
	if (CamToCalib->connected)
	{
		CamToCalib->SetCalibrationSetting(CameraMatrix, distanceCoefficients);
		if (resolution != CamToCalib->GetCameraSettings().Resolution)
		{
			cerr << "WARNING : Resolution of the stored images isn't the same as the resolution of the live camera!" <<endl;
		}
		
	}
	else
	{
		auto CamSett = CamToCalib->GetCameraSettings();
		CamSett.Resolution = resolution;
		CamSett.CameraMatrix = CameraMatrix;
		CamSett.distanceCoeffs = distanceCoefficients;
		CamSett.DeviceInfo.device_description = "NoCam";
		CamToCalib->SetCameraSetting(CamSett);
	}
	
	
	auto calconf = GetCalibrationConfig();
	double apertureWidth = calconf.SensorSize.width, apertureHeight = calconf.SensorSize.height, fovx, fovy, focalLength, aspectRatio;
	Point2d principalPoint;
	calibrationMatrixValues(CameraMatrix, CamToCalib->GetCameraSettings().Resolution, apertureWidth, apertureHeight, fovx, fovy, focalLength, principalPoint, aspectRatio);
	cout << "Computed camera parameters for sensor of size " << apertureWidth << "x" << apertureHeight <<"mm :" << endl
	<< " fov:" << fovx << "x" << fovy << "°, focal length=" << focalLength << ", aspect ratio=" << aspectRatio << endl
	<< "Principal point @ " << principalPoint << endl;
	writeCameraParameters(CamToCalib->GetCameraSettings().DeviceInfo.device_description, CameraMatrix, distanceCoefficients, CamToCalib->GetCameraSettings().Resolution);
	//distanceCoefficients = Mat::zeros(8, 1, CV_64F);
	ShowUndistorted = true;
	Calibrating = false;
}

bool docalibration(CameraSettings CamSett)
{
	bool HasCamera = CamSett.IsValid();
	CamSett.BufferSize = 1;

	CamToCalib = new VideoCaptureCamera(CamSett);
	if (HasCamera)
	{
		CamToCalib->StartFeed();
	}

	bool AutoCapture = false;
	float AutoCaptureFramerate = 2;
	double AutoCaptureStart;
	int LastAutoCapture;

	
	fs::create_directory(TempImgPath);

	namedWindow(CalibWindowName, WINDOW_NORMAL);
	setWindowProperty(CalibWindowName, WND_PROP_FULLSCREEN, WINDOW_FULLSCREEN);

	if (!HasCamera)
	{
		cout << "No camera was found, calibrating from saved images" << endl;
		CalibrationWorker();
		vector<String> pathes = CalibrationImages();
		for (int i = 0; i < pathes.size(); i++)
		{
			Mat image = imread(pathes[i]);
			UMat image2, undist;
			image.copyTo(image2);
			CamToCalib->InjectImage(0, image2);
			CamToCalib->Undistort(0);
			CamToCalib->GetFrameUndistorted(0, undist);
			imshow(CalibWindowName, undist);
			waitKey(1000);
		}
		
		return true;
	}
	CamToCalib->StartFeed();


	cout << "Camera calibration mode !" << endl
	<< "Press [space] to capture an image, [enter] to calibrate, [a] to capture an image every " << 1/AutoCaptureFramerate << "s" <<endl
	<< "Take pictures of a checkerboard with " << GetCalibrationConfig().NumIntersections.width+1 << "x" << GetCalibrationConfig().NumIntersections.height+1 << " squares of side length " << GetCalibrationConfig().SquareSideLength << "mm" << endl
	<< "Images will be saved in folder " << TempImgPath << endl;

	
	//startWindowThread();
	
	vector<String> pathes = CalibrationImages();
	int nextIdx = LastIdx(pathes) +1;
	int64 lastCapture = getTickCount();

	FrameCounter fps;
	int failed = 0;
	bool CapturedImageLastFrame = false;
	while (true)
	{
		UMat frame, frameresized;
		bool CaptureImageThisFrame = false;
		#ifdef WITH_CUDA
		cuda::GpuMat gpuframe, gpuresized;
		cuda::Stream resizestream;
		#endif
		
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
		//cout << "read success" << endl;
		//drawChessboardCorners(drawToFrame, CheckerSize, foundPoints, found);
		//char character = waitKey(1);
		if (ShowUndistorted)
		{
			UMat frameundist;
			CamToCalib->Undistort(0);

			CamToCalib->GetOutputFrame(0, frame, Rect(Point2i(0,0), CamSett.Resolution));
		}
		else
		{
			CamToCalib->GetFrame(0, frame);
		}
		
		if (GetScreenResolution() != CamSett.Resolution)
		{
			#ifdef WITH_CUDA
			gpuframe.upload(frame, resizestream);
			cuda::resize(gpuframe, gpuresized, GetScreenResolution(), 0, 0, 1, resizestream);
			gpuresized.download(frameresized);
			#else
			resize(frame, frameresized, GetScreenResolution());
			#endif
		}
		else
		{
			frameresized = frame;
		}

		switch (pollKey())
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
			if (Calibrating)
			{
				AutoCapture = false;
			}
			else if (ShowUndistorted)
			{
				ShowUndistorted = false;
			}
			else
			{
				CalibrationThread = new thread(CalibrationWorker);
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
		if (CaptureImageThisFrame && !Calibrating && !ShowUndistorted)
		{
			vector<Point2f> foundPoints;
			UMat grayscale;
			cvtColor(frame, grayscale, COLOR_BGR2GRAY);
			//bool found = checkChessboard(grayscale, CheckerSize);
			imwrite(TempImgPath + "/" + to_string(nextIdx++) + ".png", frame);
			lastCapture = getTickCount() + getTickFrequency();
			CapturedImageLastFrame = true;
		}
		else
		{
			CapturedImageLastFrame = false;
		}
		
		if (Calibrating)
		{
			putText(frameresized, "Calibrating, please wait...", Point(100,100), FONT_HERSHEY_SIMPLEX, 2, Scalar(0,255,0), 4);
		}
		else if (getTickCount() < lastCapture)
		{
			putText(frameresized, "Image " + to_string(nextIdx -1) + ( AutoCapture ? " AutoCaptured !" : " captured !"), Point(100,100), FONT_HERSHEY_SIMPLEX, 2, Scalar(255,0,0), 4);
		}
		
		
		#ifdef WITH_CUDA
		resizestream.waitForCompletion();
		#endif

		fps.AddFpsToImage(frameresized, fps.GetDeltaTime());
		imshow(CalibWindowName, frameresized);
	}
	return true;
}
