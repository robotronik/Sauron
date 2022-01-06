#include <iostream> // for standard I/O
#include <string>   // for strings
#include <iomanip>  // for controlling float print precision
#include <fstream>
#include <iosfwd>
#include <sstream>  // string to number conversion
#include <opencv2/core.hpp>     // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>  // OpenCV window I/O
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>
#include <stdio.h>
using namespace std;
using namespace cv;

const float CalibrationSquareEdge = 0.02; //m
const Size CheckerSize = Size(9, 13);

static bool readCameraParameters(std::string filename, Mat& camMatrix, Mat& distCoeffs)
{
	cv::FileStorage fs(filename, cv::FileStorage::READ);
	if (!fs.isOpened())
		return false;
	fs["camera_matrix"] >> camMatrix;
	fs["distortion_coefficients"] >> distCoeffs;
	return (camMatrix.size() == cv::Size(3,3)) ;
}

static void writeCameraParameters(std::string filename, Mat camMatrix, Mat distCoeffs)
{
	cv::FileStorage fs(filename, cv::FileStorage::WRITE);
	if (!fs.isOpened())
		return;
	fs.write("camera_matrix", camMatrix);
	fs.write("distortion_coefficients", distCoeffs);
}

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

bool SaveCameraCalibration(string name, Mat CameraMatrix, Mat distanceCoef)
{
	ofstream outStream(name);
	if (outStream)
	{
		uint16_t rows = CameraMatrix.rows;
		uint16_t columns = CameraMatrix.cols;
		for (int r = 0; r < rows; r++)
		{
			for (int c = 0; c < columns; c++)
			{
				double value = CameraMatrix.at<double>(r, c);
				outStream << value << endl;
			}
			
		}
		rows = distanceCoef.rows;
		columns = distanceCoef.cols;

		for (int r = 0; r < rows; r++)
		{
			for (int c = 0; c < columns; c++)
			{
				double value = distanceCoef.at<double>(r, c);
				outStream << value << endl;
			}
			
		}
		outStream.close();
		return true;
	}
	return false;
}

int main(int argc, char const *argv[])
{

	Mat CameraMatrix;
	Mat distanceCoefficients;

	vector<vector<Point2f>> savedImages;

	VideoCapture* vid = new VideoCapture();

	vid->open(0, CAP_ANY);
	vid->set(CAP_PROP_FRAME_WIDTH, 1920);
	vid->set(CAP_PROP_FRAME_HEIGHT, 1080);
	//vid->set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
	vid->set(CAP_PROP_FPS, 30);
	vid->set(CAP_PROP_BUFFERSIZE, 1);
	if (!vid->isOpened())
	{
		return 0;
	}

	namedWindow("Webcam", WINDOW_NORMAL);
	setWindowProperty("Webcam", WND_PROP_FULLSCREEN, WINDOW_FULLSCREEN);

	while (true)
	{
		UMat frame;
		if (!vid->read(frame))
		{
			//cout<< "read fail" <<endl;
			continue;
		}
		//cout << "read success" << endl;
		//drawChessboardCorners(drawToFrame, CheckerSize, foundPoints, found);
		char character = waitKey(1);

		switch (character)
		{
		case ' ':
			//save image
			{
				vector<Point2f> foundPoints;
				bool found = findChessboardCorners(frame, CheckerSize, foundPoints, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE | CALIB_CB_FAST_CHECK);
				drawChessboardCorners(frame, CheckerSize, foundPoints, found);
				if (found)
				{
					savedImages.push_back(foundPoints);
					imshow("Webcam", frame);
					waitKey(1000);
				}
			}
			break;
		
		case 13: //enter
			//start calib
			CameraCalibration(savedImages, CheckerSize, CalibrationSquareEdge, CameraMatrix, distanceCoefficients);
			writeCameraParameters("Calibration", CameraMatrix, distanceCoefficients);
			goto aftercalib;
			break;

		case 27:
			//exit
			return 0;
			break;

		default:
			break;
		}
		imshow("Webcam", frame);
	}

aftercalib:
	while (true)
	{
		UMat frame;
		if (!vid->read(frame))
		{
			break;
		}
		UMat undistorted;
		undistort(frame, undistorted, CameraMatrix, distanceCoefficients);
		imshow("Webcam", undistorted);
		waitKey(1);
	}
	
	
	
	return 0;
}
