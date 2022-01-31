#pragma once

#include <iostream> // for standard I/O
#include <string>   // for strings
#include <iomanip>  // for controlling float print precision
#include <sstream>  // string to number conversion
#include <opencv2/core.hpp>     // Basic OpenCV structures (Mat, Scalar)
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>  // OpenCV window I/O
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;

struct ArucoMarker
{
    float sideLength;
    int number;
    Point3d OffsetLocation;

    vector<Point3d> GetObjectPointsNoOffset()
    {
        float sql2 = sideLength*0.5;
        return {
            Point3d(-sql2, sql2, 0.0),
            Point3d(sql2, sql2, 0.0),
            Point3d(sql2, -sql2, 0.0),
            Point3d(-sql2, -sql2, 0.0)
        };
    }

    ArucoMarker()
        :sideLength(0.05),
        number(-1),
        OffsetLocation(0,0,0)
    {}

    ArucoMarker(float InSideLength, int InNumber)
        :sideLength(InSideLength),
        number(InNumber),
        OffsetLocation(0,0,0)
    {}

    ArucoMarker(float InSideLength, int InNumber, Point3d InOffsetLocation)
        :sideLength(InSideLength),
        number(InNumber),
        OffsetLocation(InOffsetLocation)
    {}
};

struct trackedobject
{
    vector<ArucoMarker> markers;

};

extern ArucoMarker center;