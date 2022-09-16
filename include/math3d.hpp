#pragma once

#include <vector>
#include <opencv2/core.hpp>

using namespace std;
using namespace cv;

Matx33d ImageToWorld();

double GetVectorLengthSquared(Vec3d x);

Vec3d NormaliseVector(Vec3d x);

//removes the component of base from x, then normalises it
//Base must be of unit length
Vec3d MakeVectorOrthogonal(Vec3d base, Vec3d x);

//makes a rotation matrix from identity to those 3 vectors
Matx33d MakeRotationFromXYZ(Vec3d X, Vec3d Y, Vec3d Z);

Matx33d MakeRotationFromXY(Vec3d X, Vec3d Y);

Matx33d MakeRotationFromZX(Vec3d Z, Vec3d X);

Matx33d MakeRotationFromZY(Vec3d Z, Vec3d Y);

Matx31d GetAxis(Matx33d rotation, int i);

double GetRotZ(Matx33d rotation);

bool ClosestPointsOnTwoLine(Vec3d Line1Orig, Vec3d Line1Dir, Vec3d Line2Orig, Vec3d Line2Dir, Vec3d& Line1Point, Vec3d& Line2Point);

void Affine3dToVictor(PositionPacket &InPacket, Affine3d position)
{
    Vec3d pos3d = position.translation() * 1000.0; //convert to mm
    InPacket.X = pos3d(0);
    InPacket.Y = pos3d(1);
    double angle = GetRotZ(position.linear()) * 180.f / M_PI;
    InPacket.rotation = angle;
}