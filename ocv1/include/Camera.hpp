#pragma once

#include <string>   // for strings
#include <opencv2/core.hpp>     // Basic OpenCV structures (Mat, Scalar)
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>  // OpenCV window I/O
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
using namespace std;
using namespace cv;

struct Camera
{
	//config
	string WindowName;
	Size CaptureSize;
	int fps;
	string DeviceID;
	int ApiID;

	//capture
	VideoCapture* feed;
	UMat frame;

	//calibration
	Mat CameraMatrix;
	Mat distanceCoeffs;

	//status
	bool physical;
	bool connected;
	bool grabbed;
	bool arucoed;

	//aruco
	vector<int> markerIDs;
	vector<vector<Point2f>> markerCorners;


	Camera(string InWindowName)
		:WindowName(InWindowName),
		physical(false),
		connected(false),
		grabbed(false)
	{}

	Camera(string InWindowName, Size InCaptureSize, int InFPS, string InDeviceID, int InApiId)
		:WindowName(InWindowName),
		CaptureSize(InCaptureSize),
		fps(InFPS),
		DeviceID(InDeviceID),
		ApiID(InApiId),
		physical(true),
		connected(false),
		grabbed(false)
	{}

	bool StartFeed()
	{
		if (!physical || connected)
		{
			return false;
		}
		feed = new VideoCapture();
		feed->open(DeviceID, ApiID);
		feed->set(CAP_PROP_FRAME_WIDTH, CaptureSize.width);
		feed->set(CAP_PROP_FRAME_HEIGHT, CaptureSize.height);
		feed->set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
		feed->set(CAP_PROP_FPS, fps);
		feed->set(CAP_PROP_BUFFERSIZE, 1);
		connected = true;
		return true;
	}

	bool Grab()
	{
		if (!physical)
		{
			return false;
		}
		if (!connected)
		{
			return false;
		}
		grabbed = feed->grab();
		return grabbed;
	}

	bool Read()
	{
		if (!physical)
		{
			return false;
		}
		if (!connected)
		{
			return false;
		}
		arucoed = false;
		if (grabbed)
		{
			grabbed = false;
			return feed->retrieve(frame);
		}
		
		return feed->read(frame);
	}

	void Show()
	{
		if (!frame.empty())
		{
			imshow(WindowName, frame);
		}
		else
		{
			Mat red = Mat(Size(192,108), CV_8UC3, Scalar(0, 0, 255));
			imshow(WindowName, red);
		}
		
	}

	void detectMarkers(Ptr<aruco::Dictionary> dict, Ptr<aruco::DetectorParameters> params)
	{
		if (frame.empty())
		{
			return;
		}
		
		aruco::detectMarkers(frame, dict, markerCorners, markerIDs, params);
		arucoed = true;
	}

	UMat GetFrame()
	{
		return frame;
	}

	UMat GetFrameDebug(Size winsize)
	{
		double fx = frame.cols / winsize.width;
		double fy = frame.rows / winsize.height;
		double fz = max(fx, fy);
		UMat resized;
		resize(frame, resized, Size(winsize.width, winsize.height), fz, fz, INTER_LINEAR);
		if (arucoed)
		{
			vector<vector<Point2f>> raruco;
			for (int i = 0; i < markerCorners.size(); i++)
			{
				vector<Point2f> marker;
				for (int j = 0; j < markerCorners[i].size(); j++)
				{
					marker.push_back(markerCorners[i][j]/fz);
				}
				raruco.push_back(marker);
			}
			aruco::drawDetectedMarkers(resized, raruco, markerIDs);
		}
		return resized;
	}

};

vector<Camera*> autoDetectCameras(int maxidx);

bool StartCameras(vector<Camera*> Cameras);

static bool readCameraParameters(std::string filename, Mat& camMatrix, Mat& distCoeffs);