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

#include "Camera.hpp"
#include "Calibfile.hpp"
#include "FrameCounter.hpp"

using namespace std;
using namespace cv;
namespace fs = std::filesystem;

const float CalibrationSquareEdge = 0.02; //m
const Size CheckerSize = Size(9, 9);
const String TempImgPath = "TempCalib";


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

void CameraCalibration(vector<vector<Point2f>> CheckerboardImageSpacePoints, Size BoardSize, float SquareEdgeLength, Mat& CameraMatrix, Mat& DistanceCoefficients)
{
	vector<vector<Point3f>> WorldSpaceCornerPoints(1);
	CreateKnownBoardPos(BoardSize, SquareEdgeLength, WorldSpaceCornerPoints[0]);
	WorldSpaceCornerPoints.resize(CheckerboardImageSpacePoints.size(), WorldSpaceCornerPoints[0]);

	vector<Mat> rVectors, tVectors;
	CameraMatrix = Mat::eye(3, 3, CV_64F);
	DistanceCoefficients = Mat::zeros(8, 1, CV_64F);
	
	calibrateCamera(WorldSpaceCornerPoints, CheckerboardImageSpacePoints, BoardSize, CameraMatrix, DistanceCoefficients, rVectors, tVectors);
}

vector<String> CalibrationImages()
{
	vector<String> pathos;
	for (const auto & entry : fs::directory_iterator(TempImgPath))
    {
		pathos.push_back(entry.path().string());
		cout << entry.path().string() << endl;
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

void ReadAndCalibrate(Mat& CameraMatrix, Mat& DistanceCoefficients)
{
	vector<String> pathes = CalibrationImages();
	vector<vector<Point2f>> savedPoints;
	for (int i = 0; i < pathes.size(); i++)
	{
		Mat frame = imread(pathes[i], IMREAD_GRAYSCALE);
		vector<Point2f> foundPoints;
		bool found = findChessboardCorners(frame, CheckerSize, foundPoints, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE | CALIB_CB_FAST_CHECK);
		if (found)
		{
			TermCriteria criteria(TermCriteria::COUNT | TermCriteria::EPS, 30, 0.001);
			cornerSubPix(frame, foundPoints, Size(11,11), Size(-1,-1), criteria);
			//Scalar sharpness = estimateChessboardSharpness(frame, CheckerSize, foundPoints);
			savedPoints.push_back(foundPoints);
		}
	}
	CameraCalibration(savedPoints, CheckerSize, CalibrationSquareEdge, CameraMatrix, DistanceCoefficients);
}

bool docalibration(Camera* CamToCalib)
{

	Mat CameraMatrix;
	Mat distanceCoefficients;

	bool ShowUndistorted = false;
	CamToCalib->StartFeed();

	fs::create_directory(TempImgPath);

	namedWindow("Webcam", WINDOW_NORMAL);
	setWindowProperty("Webcam", WND_PROP_FULLSCREEN, WINDOW_FULLSCREEN);
	
	vector<String> pathes = CalibrationImages();
	int nextIdx = LastIdx(pathes) +1;

	FrameCounter fps;
	FrameCounter readget, drawfps, showimg;
	while (true)
	{
		UMat frame;
		readget.GetDeltaTime();
		if (!CamToCalib->Read(0))
		{
			//cout<< "read fail" <<endl;
			continue;
		}
		CamToCalib->GetFrame(0, frame);
		double readgetdelta = readget.GetDeltaTime();
		//cout << "read success" << endl;
		//drawChessboardCorners(drawToFrame, CheckerSize, foundPoints, found);
		char character = waitKey(1);

		if (ShowUndistorted)
		{
			undistort(frame, frame, CameraMatrix, distanceCoefficients);
		}

		switch (character)
		{
		case ' ':
			//save image
			if (!ShowUndistorted)
			{
				vector<Point2f> foundPoints;
				UMat grayscale;
				cvtColor(frame, grayscale, COLOR_BGR2GRAY);
				bool found = findChessboardCorners(grayscale, CheckerSize, foundPoints, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE | CALIB_CB_FAST_CHECK);
				
				if (found)
				{
					imwrite(TempImgPath + "/" + to_string(nextIdx++) + ".png", frame);
					/*TermCriteria criteria(TermCriteria::COUNT | TermCriteria::EPS, 30, 0.001);
	      			cornerSubPix(grayscale, foundPoints, Size(11,11), Size(-1,-1), criteria);
					Scalar sharpness = estimateChessboardSharpness(frame, CheckerSize, foundPoints);
					putText(frame, to_string(sharpness[0]), Point2i(0, frame.rows-20), FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 0, 0));
					if (sharpness[0] < 4.f || true)
					{
						savedPoints.push_back(foundPoints);
					}*/
				}
				drawChessboardCorners(frame, CheckerSize, foundPoints, found);
				imshow("Webcam", frame);
				timespec waitFor;
				waitFor.tv_sec = 1;
				waitFor.tv_nsec = 0;
				nanosleep(&waitFor, NULL);
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
				ReadAndCalibrate(CameraMatrix, distanceCoefficients);
				writeCameraParameters(CamToCalib->GetName(), CameraMatrix, distanceCoefficients);
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
		drawfps.GetDeltaTime();
		fps.AddFpsToImage(frame, fps.GetDeltaTime());
		double drawfpsdelta = drawfps.GetDeltaTime();
		showimg.GetDeltaTime();
		imshow("Webcam", frame);
		double showdelta = showimg.GetDeltaTime();

		cout << "get : " << readgetdelta << " drawfps : " << drawfpsdelta << " show : " << showdelta << endl;
	}
	return true;
}
