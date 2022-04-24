#include "visualisation/BoardViz2D.hpp"

#include <math.h>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudawarping.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/viz.hpp>

#include "data/FrameCounter.hpp"
#include "math3d.hpp"

using namespace cv;


const String assetpath = "../assets/";

bool BoardViz2D::ImagesLoaded;
cuda::GpuMat BoardViz2D::table;
cuda::GpuMat BoardViz2D::robot;
cuda::GpuMat BoardViz2D::rouge, BoardViz2D::vert, BoardViz2D::bleu, BoardViz2D::autre;
cuda::GpuMat BoardViz2D::camera;

void BoardViz2D::LoadImage(cuda::GpuMat& output, String location)
{
	output.upload(imread(assetpath + location, IMREAD_UNCHANGED));
}

FVector2D<int> BoardViz2D::BoardToPixel(FVector2D<float> location)
{
	return (location+Center)/Extent*FVector2D<int>(image.size());
}

FVector2D<float> BoardViz2D::GetImageCenter(cuda::GpuMat& Image)
{
	return FVector2D<float>((Image.cols-1)/2.f, (Image.rows-1)/2.f);
}

void BoardViz2D::InitImages()
{
	if (ImagesLoaded)
	{
		return;
	}
	ImagesLoaded = true;
	LoadImage(table, "VisuelTable.png");
	LoadImage(robot, "Robot.png");
	LoadImage(rouge, "rouge.png");
	LoadImage(vert, "vert.png");
	LoadImage(bleu, "bleu.png");
	LoadImage(autre, "autre.png");
	LoadImage(camera, "camera.png");
}

void BoardViz2D::CreateBackground(Size Resolution)
{
	cuda::resize(table, image, Resolution);
}

void BoardViz2D::GetImage(UMat& rvalue)
{
	image.download(rvalue);
}

FVector2D<float> RotatePointAround(FVector2D<float> center, FVector2D<float> point, float rotation)
{
	FVector2D<float> diff = point-center;
	float angle = atan2(diff.y, diff.x) + rotation;
	float magnitude = sqrt(diff.x*diff.x+diff.y*diff.y);
	double x, y;
	sincos(angle, &y, &x);
	return FVector2D<float>(x,y)*magnitude + center;
}

void BoardViz2D::OverlayImage(cuda::GpuMat& ImageToOverlay, FVector2D<float> position, float rotation, FVector2D<float> ImageSize)
{

	FVector2D<int> BoardLocation = BoardToPixel(position);
	FVector2D<float> width = FVector2D<float>(ImageToOverlay.cols, 0);
	FVector2D<float> height = FVector2D<float>(0, ImageToOverlay.rows);
	Point2f srcTri[3];
    srcTri[0] = GetImageCenter(ImageToOverlay);
    srcTri[1] = FVector2D<float>(srcTri[0]) - width;
    srcTri[2] = FVector2D<float>(srcTri[0]) - height;

	FVector2D<float> TargetSize = ImageSize/Extent*FVector2D<int>(image.size());
	FVector2D<int> Oversize = TargetSize * sqrt(2); //oversize to guarantee no cropping
	cuda::GpuMat OverlayRotated(Oversize.ToSize(), ImageToOverlay.type());

    Point2f dstTri[3];
    dstTri[0] = GetImageCenter(OverlayRotated);
    dstTri[1] = RotatePointAround(dstTri[0], FVector2D<float>(dstTri[0]) - FVector2D<float>(TargetSize.x,0), rotation);
    dstTri[2] = RotatePointAround(dstTri[0], FVector2D<float>(dstTri[0]) - FVector2D<float>(0,TargetSize.y), rotation);
	Mat warp_mat = getAffineTransform( srcTri, dstTri );

	cuda::warpAffine(ImageToOverlay, OverlayRotated, warp_mat, OverlayRotated.size());

	FVector2D<int> halfsize = Oversize/2;
	
	FVector2D<int> ROIStart = BoardLocation - halfsize;
	ROIStart = FVector2D<int>::Clamp(ROIStart, FVector2D<int>(0), FVector2D<int>(image.size()));
	FVector2D<int> ROIEnd = BoardLocation - halfsize + Oversize;
	ROIEnd = FVector2D<int>::Clamp(ROIEnd, FVector2D<int>(0), FVector2D<int>(image.size()));

	cv::Rect ROIImage((Point2i)ROIStart, (Point2i)ROIEnd);
	cv::Rect ROIrobot((Point2i)(ROIStart-BoardLocation+halfsize), (Point2i)(ROIEnd-BoardLocation+halfsize));

	if (ROIImage.height == 0 || ROIImage.width == 0)
	{
		return;
	}
	

	//printf("img1 type = %d, img2 type = %d should be %d %d %d %d\n", OverlayRotated.type(), image.type(), CV_8UC4, CV_16UC4, CV_32SC4, CV_32FC4);

	cuda::alphaComp(cuda::GpuMat(OverlayRotated, ROIrobot), cuda::GpuMat(image, ROIImage), cuda::GpuMat(image, ROIImage), cuda::ALPHA_OVER);

	//cuda::alphaComp(OverlayRotated, image, image, cuda::ALPHA_OVER);
}

void BoardViz2D::OverlayImage(cuda::GpuMat& ImageToOverlay, Affine3d position, FVector2D<float> ImageSize)
{
	Vec3d translation3d;
	position.translation(translation3d);
	FVector2D<float> translation2d(translation3d[0], translation3d[1]);
	float rot = GetRotZ(position.rotation());
	OverlayImage(ImageToOverlay, translation2d, rot, ImageSize);
}

FVector2D<float> BoardViz2D::GetExtent()
{
	return Extent;
}
FVector2D<float> BoardViz2D::GetCenter()
{
	return Center;
}

cuda::GpuMat& BoardViz2D::GetRobotImage()
{
	return robot;
}

cuda::GpuMat& BoardViz2D::GetPalet(PaletCouleur type)
{
	switch (type)
	{
	case PaletCouleur::rouge :
		return rouge;
	case PaletCouleur::vert :
		return vert;
	case PaletCouleur::bleu :
		return bleu;
	default:
	case PaletCouleur::autre :
		return autre;
		break;
	}
}

cuda::GpuMat& BoardViz2D::GetCamera()
{
	return camera;
}

void BoardViz2D::GetOutputFrame(int BufferIndex, UMat& OutFrame, Size winsize)
{
	cuda::GpuMat resizedgpu;

	double fx = image.cols / winsize.width;
	double fy = image.rows / winsize.height;
	double fz = max(fx, fy);

	cuda::resize(image, resizedgpu, Size(winsize.width, winsize.height), fz, fz, INTER_LINEAR);
	cuda::cvtColor(resizedgpu, resizedgpu, COLOR_BGRA2BGR);
	resizedgpu.download(OutFrame);
	//cout << "Resize OK" <<endl;
}

void TestBoardViz()
{
	BoardViz2D::InitImages();
	BoardViz2D board(FVector2D<float>(3.0f, 2.0f), FVector2D<float>(1.5f, 1.0f));
	FVector2D<float> pos = board.GetCenter() + board.GetExtent();
	double rot = 0;
	FrameCounter fps;
	while (waitKey(1) < 0)
	{
		board.CreateBackground(Size(1500, 1000));
		board.OverlayImage(board.GetPalet(PaletCouleur::rouge), FVector2D<float>(0.1,0.2), M_PI/4, FVector2D<float>(0.15));
		FVector2D<float> actualpos = (pos - board.GetExtent()).Abs() - board.GetCenter();
		board.OverlayImage(board.GetRobotImage(), actualpos, rot, FVector2D<float>(0.42));
		double deltaTime = fps.GetDeltaTime();
		pos = (pos + FVector2D<float>(1.0,1.0) * deltaTime) % (board.GetExtent()*2);
		rot = fmod(rot + deltaTime * M_PI, 2*M_PI);
		cv::UMat imagecpu; 
		board.GetImage(imagecpu);
		fps.AddFpsToImage(imagecpu, deltaTime);
		imshow("Boardviz", imagecpu);
	}
	exit(EXIT_SUCCESS);
}