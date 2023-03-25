#include <string>
#include <opencv2/core.hpp>
#include <iostream>


static bool readCameraParameters(std::string filename, cv::Mat& camMatrix, cv::Mat& distCoeffs, cv::Size Resolution)
{
	cv::FileStorage fs(filename + ".yaml", cv::FileStorage::READ);
	if (!fs.isOpened())
	{
		//std::cerr << "Failed to read camera parameters for " << filename << std::endl;
		return false;
	}
	
	cv::Mat1i resmat; cv::Size CalibRes;
	fs["resolution"] >> resmat;
	CalibRes.width = resmat.at<int>(0,0);
	CalibRes.height = resmat.at<int>(0,1);
	cv::Mat calibmatrix;
	fs["camera_matrix"] >> calibmatrix;
	fs["distortion_coefficients"] >> distCoeffs;
	if (Resolution.area() != 0)
	{
		cv::Mat scalingMatrix = cv::Mat::zeros(3,3, CV_64F);
		scalingMatrix.at<double>(0,0) = (double)Resolution.width / CalibRes.width;
		scalingMatrix.at<double>(1,1) = (double)Resolution.height / CalibRes.height;
		scalingMatrix.at<double>(2,2) = 1;
		camMatrix = scalingMatrix * calibmatrix;
	}
	else
	{
		camMatrix = calibmatrix;
	}
	
	
	
	//cout << "Calibration file resolution = " << CalibRes << endl;
	//cout << "Reading at resolution " <<Resolution << endl;
	//cout << scalingMatrix << " * " << calibmatrix << " = " << camMatrix << endl;
	return (camMatrix.size() == cv::Size(3,3));
}

static void writeCameraParameters(std::string filename, cv::Mat camMatrix, cv::Mat distCoeffs, cv::Size Resolution)
{
	cv::FileStorage fs(filename + ".yaml", cv::FileStorage::WRITE);
	if (!fs.isOpened())
	{
		return;
	}
	cv::Mat1i resmat(1,2);
	resmat.at<int>(0,0) = Resolution.width;
	resmat.at<int>(0,1) = Resolution.height;
	fs.write("resolution", resmat);
	fs.write("camera_matrix", camMatrix);
	fs.write("distortion_coefficients", distCoeffs);
}