#include <string>
#include <opencv2/core.hpp>
#include <fstream>


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