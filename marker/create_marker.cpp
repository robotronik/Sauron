#include <iostream> // for standard I/O
#include <string>   // for strings
#include <iomanip>  // for controlling float print precision
#include <sstream>  // string to number conversion
#include <opencv2/core.hpp>     // Basic OpenCV structures (Mat, Scalar)
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>  // OpenCV window I/O
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <stdio.h>
using namespace std;
using namespace cv;

int main(int argc, char const *argv[])
{
    
    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_100);
    for (int i = 0; i < 100; i++)
    {
        Mat markerImage;
        aruco::drawMarker(dictionary, i, 1024, markerImage, 1);
        char buffer[30];
        snprintf(buffer, sizeof(buffer), "markers/marker%d.png", i);
        imwrite(buffer, markerImage);
    }
    Ptr<aruco::Dictionary> dictionary2 = aruco::getPredefinedDictionary(aruco::DICT_4X4_100);
    Ptr<aruco::CharucoBoard> board = aruco::CharucoBoard::create(19, 10, 0.04f, 0.03f, dictionary2);
    Mat CalibrationBoard;
    board->draw(Size(1920, 1080), CalibrationBoard, 10, 1);
    imwrite("Calibration.png", CalibrationBoard);
    return 0;
}
