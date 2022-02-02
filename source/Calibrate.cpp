#include "Calibrate.hpp"

#include <iostream> // for standard I/O
#include <string>   // for strings
#include <fstream>
#include <iosfwd>
#include <stdio.h>
#include <sstream>  // string to number conversion
#include <chrono> //seconds, micros, nanos
#include <thread> //sleep_for
#include <unistd.h>

#include <opencv2/core.hpp>     // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>  // OpenCV window I/O
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>

#include "Camera.hpp"
#include "Calibfile.hpp"
#include "FrameCounter.hpp"

using namespace std;
using namespace cv;

const float CalibrationSquareEdge = 0.02; //m
const Size CheckerSize = Size(9, 13);

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

bool docalibration(Camera* CamToCalib)
{

	Mat CameraMatrix;
	Mat distanceCoefficients;

	vector<vector<Point2f>> savedPoints;
	CamToCalib->StartFeed();

	namedWindow("Webcam", WINDOW_NORMAL);
	setWindowProperty("Webcam", WND_PROP_FULLSCREEN, WINDOW_FULLSCREEN);

	FrameCounter fps;
	while (true)
	{
		UMat frame;
		if (!CamToCalib->Read())
		{
			//cout<< "read fail" <<endl;
			continue;
		}
		CamToCalib->GetFrame(frame);
		//cout << "read success" << endl;
		//drawChessboardCorners(drawToFrame, CheckerSize, foundPoints, found);
		char character = waitKey(1);

		switch (character)
		{
		case ' ':
			//save image
			{
				vector<Point2f> foundPoints;
				UMat grayscale;
				cvtColor(frame, grayscale, COLOR_BGR2GRAY);
				bool found = findChessboardCorners(grayscale, CheckerSize, foundPoints, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE | CALIB_CB_FAST_CHECK);
				drawChessboardCorners(frame, CheckerSize, foundPoints, found);
				if (found)
				{
					TermCriteria criteria(TermCriteria::COUNT | TermCriteria::EPS, 30, 0.001);
	      			cornerSubPix(grayscale, foundPoints, Size(11,11), Size(-1,-1), criteria);
					Scalar sharpness = estimateChessboardSharpness(frame, CheckerSize, foundPoints);
					putText(frame, to_string(sharpness[0]), Point2i(0, frame.rows-20), FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 0, 0));
					if (sharpness[0] < 4.f || true)
					{
						savedPoints.push_back(foundPoints);
					}
				}
				imshow("Webcam", frame);
				this_thread::sleep_for(chrono::seconds(1));
			}
			break;
		
		case 13: //enter
			//start calib
			CameraCalibration(savedPoints, CheckerSize, CalibrationSquareEdge, CameraMatrix, distanceCoefficients);
			writeCameraParameters(CamToCalib->GetName(), CameraMatrix, distanceCoefficients);
			goto aftercalib;
			break;

		case 27:
			//exit
			return false;
			break;

		default:
			break;
		}
		fps.AddFpsToImage(frame, fps.GetDeltaTime());
		imshow("Webcam", frame);
	}

aftercalib:
	while (true)
	{
		UMat frame;
		if (!CamToCalib->Read())
		{
			//cout<< "read fail" <<endl;
			continue;
		}
		CamToCalib->GetFrame(frame);
		UMat undistorted;
		undistort(frame, undistorted, CameraMatrix, distanceCoefficients);
		fps.AddFpsToImage(frame, fps.GetDeltaTime());
		imshow("Webcam", undistorted);
		waitKey(1);
	}
	return true;
}
