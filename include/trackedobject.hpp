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
#include <opencv2/core/affine.hpp>

class Camera;

using namespace std;
using namespace cv;

struct ArucoMarker
{
    float sideLength;
    int number;
    Affine3d Pose;

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
        Pose(Affine3d::Identity())
    {}

    ArucoMarker(float InSideLength, int InNumber)
        :sideLength(InSideLength),
        number(InNumber),
        Pose(Affine3d::Identity())
    {}

    ArucoMarker(float InSideLength, int InNumber, Affine3d InPose)
        :sideLength(InSideLength),
        number(InNumber),
        Pose(InPose)
    {}
};

struct trackedobject
{
    vector<ArucoMarker> markers;

};

extern ArucoMarker center;

vector<Point2f> ReorderMarkerCorners(vector<Point2f> Corners);

Affine3d GetTagTransform(ArucoMarker& Tag, std::vector<Point2f> Corners, Camera* Cam);

Affine3d GetTransformRelativeToTag(ArucoMarker& Tag, std::vector<Point2f> Corners, Camera* Cam);