#include "trackedobject.hpp"

extern ArucoMarker center(0.1, 42, Point3d(0,0,0.25));

vector<Point2f> ReorderMarkerCorners(vector<Point2f> Corners)
{
    vector<Point2f> newCorners{Corners[3], Corners[2], Corners[1], Corners[0]};
    return newCorners;
}