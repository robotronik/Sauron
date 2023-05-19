/* 

	Ce fichier est une bibliothèque de fonctions mathématiques 3D. Il utilise la bibliothèque OpenCV, une bibliothèque d'outils et d'algorithme de vision par ordinateur. Ainsi que la bibliothèque glm, une bibliothèque mathématique pour les graphiques 3D.

*/

#include "math3d.hpp"
#include <math.h>
#include <glm/glm.hpp>

using namespace cv;
using namespace std;


// Adapte la taille d'une image à une taille cible en gardant le ratio de l'image
Size ScaleToFit(Size original, Size target)
{
	Size &insize = target;
	Size &bssize = original;
	int wmax = bssize.width*insize.height/bssize.height;
	wmax = min(insize.width, wmax);
	int hmax = bssize.height*insize.width/bssize.width;
	hmax = min(insize.height, hmax);
	
	Size outsize(wmax, hmax);
	return outsize;
}

// Surcharge de la fonction précédente pour adapter une image à un rectangle et retourne ce rectangle
cv::Rect ScaleToFit(cv::Size original, cv::Rect target)
{
	Size targetsz = ScaleToFit(original, target.size());
	Size szdiff = target.size()-targetsz;
	Rect roi(szdiff.width/2 + target.x,szdiff.height/2 + target.y, targetsz.width, targetsz.height);
	return roi;
}

// Cette fonction calcule le champ de vision de la caméra (en radians) à partir de la matrice de la caméra et de la résolution de l'image
Size2d GetCameraFOV(Size Resolution, Mat CameraMatrix)
{
	if (CameraMatrix.size().width != 3 || CameraMatrix.size().height != 3)
	{
		return Size2d(M_PI/2, M_PI/2);
	}
	double fx = CameraMatrix.at<double>(0,0), fy = CameraMatrix.at<double>(1,1);
	double fovx = 2*atan(Resolution.width/(2*fx)), fovy = 2*atan(Resolution.height/(2*fy));
	return Size2d(fovx, fovy);
} 

// Cette fonction renvoie une matrice de transformation d'espace qui convertit les coordonnées de l'image en coordonnées du monde réel
Matx33d ImageToWorld()
{
	return Matx33d(1,0,0, 0,-1,0, 0,0,1);
}

// Retourne la longueur carrée d'un vecteur
double GetVectorLengthSquared(Vec3d x)
{
	return x.ddot(x);
}

// Retourne la longueur d'un vecteur normalisé
Vec3d NormaliseVector(Vec3d x)
{
	return x / sqrt(GetVectorLengthSquared(x));
}

// Retourne un vecteur orthogonal à un vecteur de base
Vec3d MakeVectorOrthogonal(Vec3d base, Vec3d x)
{
	double comp = x.ddot(base);
	Vec3d rem = x-base*comp;
	return NormaliseVector(rem);
}

// Retourne une matrice de rotation à partir de trois vecteurs
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

// Retourne une matrice de rotation à partir de deux vecteurs
Matx33d MakeRotationFromXY(Vec3d X, Vec3d Y)
{
	X = NormaliseVector(X);
	Y = MakeVectorOrthogonal(X, Y);
	Vec3d Z = X.cross(Y);
	return MakeRotationFromXYZ(X, Y, Z);
}


// Retourne une matrice de rotation à partir de deux vecteurs
Matx33d MakeRotationFromXZ(Vec3d X, Vec3d Z)
{
	X = NormaliseVector(X);
	Z = MakeVectorOrthogonal(X, Z);
	Vec3d Y = Z.cross(X);
	return MakeRotationFromXYZ(X, Y, Z);
}

// Retourne une matrice de rotation à partir de deux vecteurs
Matx33d MakeRotationFromZX(Vec3d Z, Vec3d X)
{
	Z = NormaliseVector(Z);
	X = MakeVectorOrthogonal(Z, X);
	Vec3d Y = Z.cross(X);
	return MakeRotationFromXYZ(X, Y, Z);
}

// Retourne une matrice de rotation à partir de deux vecteurs
Matx33d MakeRotationFromZY(Vec3d Z, Vec3d Y)
{
	Z = NormaliseVector(Z);
	Y = MakeVectorOrthogonal(Z, Y);
	Vec3d X = Y.cross(Z);
	return MakeRotationFromXYZ(X, Y, Z);
}

// Retourne une matrice de rotation à partir de deux vecteurs
Matx31d GetAxis(Matx33d rotation, int i)
{
	return rotation.col(i);
}

// Retourne l'angle de rotation autour de l'axe Z
double GetRotZ(Matx33d rotation)
{
	Matx31d Xaxis = GetAxis(rotation, 0);
	Xaxis(2,0) = 0; //project on XY
	Xaxis /= sqrt(Xaxis.ddot(Xaxis));
	return atan2(Xaxis(1,0), Xaxis(0,0));
}

// Projette un point sur une ligne donnée
Vec3d ProjectPointOnLine(Vec3d Point, Vec3d LineOrig, Vec3d LineDir)
{
	Vec3d LineToPoint = Point-LineOrig;
	double dist = LineDir.ddot(LineToPoint);
	Vec3d projected = LineOrig + dist * LineDir;
	return projected;
}

// Cette fonction trouve les points les plus proches sur deux lignes
bool ClosestPointsOnTwoLine(Vec3d Line1Orig, Vec3d Line1Dir, Vec3d Line2Orig, Vec3d Line2Dir, Vec3d &Line1Point, Vec3d &Line2Point)
{
	/*https://math.stackexchange.com/questions/846054/closest-points-on-two-line-segments*/

	// Algorithm is ported from the C algorithm of 
	// Paul Bourke at http://local.wasp.uwa.edu.au/~pbourke/geometry/lineline3d/

	Vec3d p13 = Line1Orig - Line2Orig;
	Vec3d p43 = Line2Dir;

	/*if (p43.LengthSq() < Math.Epsilon) {
		return false;
	}*/
	Vec3d p21 = Line1Dir;
	/*if (p21.LengthSq() < Math.Epsilon) {
		return false;
	}*/

	double d1343 = p13.ddot(p43);
	double d4321 = p43.ddot(p21);
	double d1321 = p13.ddot(p21);
	double d4343 = p43.ddot(p43);
	double d2121 = p21.ddot(p21);

	double denom = d2121 * d4343 - d4321 * d4321;
	if (abs(denom) < 1e-9) {
		return false;
	}
	double numer = d1343 * d4321 - d1321 * d4343;

	double mua = numer / denom;
	double mub = (d1343 + d4321 * (mua)) / d4343;

	Line1Point = Line1Orig + mua*p21;
	Line2Point = Line2Orig + mub*p43;

	return true;


	/*Vec3d V21 = Line2Orig - Line1Orig;
	double v11 = Line1Dir.ddot(Line1Dir);
	double v22 = Line2Dir.ddot(Line2Dir);
	double v21 = Line2Dir.ddot(Line1Dir);
	double v21_1 = V21.ddot(Line1Dir);
	double v21_2 = V21.ddot(Line2Dir);
	double denom = v21 * v21 - v22 * v11;
	double s = 0, t = 0;
	if (abs(denom) < 1e-9)
	{
		return false;
	}
	s = (v21_2 * v21 - v22 * v21_1) / denom;
	t = (-v21_1 * v21 + v11 * v21_2) / denom;

	Line1Point = Line1Orig + s * Line1Dir;
	Line2Point = Line2Orig + t * Line2Dir;
	return true;*/
}

// Cette fonction convertit une transformation affine 3D (rotation + translation) de OpenCV en matrice 4x4 de glm
glm::mat4 Affine3DToGLM(cv::Affine3d Location)
{
	glm::mat4 outmat;
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			outmat[j][i] = Location.matrix(i,j);
		}
	}
	return outmat;
}