#include "math3d.hpp"

#include <math.h>

Matx33d ImageToWorld()
{
	return Matx33d(1,0,0, 0,-1,0, 0,0,1);
}

double GetVectorLengthSquared(Vec3d x)
{
    return x.ddot(x);
}

Vec3d NormaliseVector(Vec3d x)
{
    return x / sqrt(GetVectorLengthSquared(x));
}

Vec3d MakeVectorOrthogonal(Vec3d base, Vec3d x)
{
    double comp = x.ddot(base);
    Vec3d rem = x-base*comp;
    return NormaliseVector(rem);
}

Matx33d MakeRotationFromXYZ(Vec3d X, Vec3d Y, Vec3d Z)
{
    Matx33d outputmatrix;
    //ensure orthonormal
    X = NormaliseVector(X); 
    Y = MakeVectorOrthogonal(X, Y); 
    Z = MakeVectorOrthogonal(Y, MakeVectorOrthogonal(X, Z));

    Vec3d vectors[3] = {X, Y, Z};
    for (int i = 0; i < 3; i++)
    {
        //outputmatrix.col(i) = vectors[i];
        for (int j = 0; j < 3; j++)
        {
            outputmatrix(j,i) = vectors[i](j);
        }
        
    }
    return outputmatrix;
}

Matx33d MakeRotationFromZX(Vec3d Z, Vec3d X)
{
    Z = NormaliseVector(Z);
    X = MakeVectorOrthogonal(Z, X);
    Vec3d Y = Z.cross(X);
    return MakeRotationFromXYZ(X, Y, Z);
}

Matx33d MakeRotationFromZY(Vec3d Z, Vec3d Y)
{
    Z = NormaliseVector(Z);
    Y = MakeVectorOrthogonal(Z, Y);
    Vec3d X = Y.cross(Z);
    return MakeRotationFromXYZ(X, Y, Z);
}