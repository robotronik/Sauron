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

Matx33d MakeRotationFromZX(Vec3d Z, Vec3d X);

Matx33d MakeRotationFromZY(Vec3d Z, Vec3d Y);