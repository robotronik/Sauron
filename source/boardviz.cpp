#include "boardviz.hpp"

#include <math.h>
//#include <cmath>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudawarping.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudaimgproc.hpp>

#include "FrameCounter.hpp"

using namespace cv;

cuda::GpuMat table;
cuda::GpuMat robot;

FVector2D<float> TailleTable;
FVector2D<float> CentreTable;

cuda::GpuMat image;

void InitImages()
{
	Mat tablecpu = imread("../assets/VisuelTable.png", IMREAD_UNCHANGED);
	table.upload(tablecpu);
	Mat robotcpu = imread("../assets/Robot.png", IMREAD_UNCHANGED);
	robot.upload(robotcpu);
}

void CreateSpace(FVector2D<float> extent, FVector2D<float> center)
{
	TailleTable = extent;
	CentreTable = center;
}

void CreateBackground(Size Resolution)
{
	cuda::resize(table, image, Resolution);
}

void GetImage(UMat& rvalue)
{
	image.download(rvalue);
}

void TestBoardViz()
{
	InitImages();
	CreateSpace(FVector2D<float>(3.0f, 2.0f), FVector2D<float>(1.5f, 1.0f));
	FVector2D<float> pos = CentreTable + TailleTable;
	double rot = 0;
	FrameCounter fps;
	while (waitKey(1) < 0)
	{
		CreateBackground(Size(1500, 1000));
		FVector2D<float> actualpos = (pos - TailleTable).Abs() - CentreTable;
		OverlayImage(robot, actualpos, rot, FVector2D<float>(0.1));
		double deltaTime = fps.GetDeltaTime();
		pos = (pos + FVector2D<float>(1.0,1.0) * deltaTime) % (TailleTable*2);
		rot = fmod(rot + deltaTime * M_PI, 2*M_PI);
		cv::UMat imagecpu; 
		GetImage(imagecpu);
		fps.AddFpsToImage(imagecpu, deltaTime);
		imshow("Boardviz", imagecpu);
	}
	exit(EXIT_SUCCESS);
}

FVector2D<int> BoardToPixel(FVector2D<float> location)
{
	return (location+CentreTable)/TailleTable*FVector2D<int>(image.size());
}

FVector2D<float> GetImageCenter(cuda::GpuMat& Image)
{
	return FVector2D<float>((Image.cols-1)/2.f, (Image.rows-1)/2.f);
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

void OverlayImage(cuda::GpuMat& ImageToOverlay, FVector2D<float> position, float rotation, FVector2D<float> ImageSize)
{
	cuda::GpuMat OverlayRotated(image.size(), ImageToOverlay.type());

	FVector2D<int> BoardLocation = BoardToPixel(position);
	FVector2D<float> width = FVector2D<float>(ImageToOverlay.cols, 0);
	FVector2D<float> height = FVector2D<float>(0, ImageToOverlay.rows);
	Point2f srcTri[3];
    srcTri[0] = GetImageCenter(ImageToOverlay);
    srcTri[1] = FVector2D<float>(srcTri[0]) - width/2.f;
    srcTri[2] = FVector2D<float>(srcTri[0]) - height/2.f;
    Point2f dstTri[3];
    dstTri[0] = FVector2D<float>(BoardLocation);
    dstTri[1] = RotatePointAround(dstTri[0], FVector2D<float>(dstTri[0]) - FVector2D<float>(ImageSize.x/TailleTable.x * image.cols,0), rotation);
    dstTri[2] = RotatePointAround(dstTri[0], FVector2D<float>(dstTri[0]) - FVector2D<float>(0,ImageSize.y/TailleTable.y * image.rows), rotation);
	Mat warp_mat = getAffineTransform( srcTri, dstTri );

	cuda::warpAffine(ImageToOverlay, OverlayRotated, warp_mat, OverlayRotated.size());

	/*FVector2D<int> halfsize = FVector2D<int>(OverlayRotated.size())/2;
	
	FVector2D<int> ROIStart = BoardLocation - halfsize;
	ROIStart = FVector2D<int>::Max(ROIStart, FVector2D<int>(0));
	FVector2D<int> ROIEnd = BoardLocation + halfsize;
	ROIEnd = FVector2D<int>::Min(ROIEnd, FVector2D<int>(image.size()));

	cv::Rect ROIImage((Point2i)ROIStart, (Point2i)ROIEnd);
	cv::Rect ROIrobot((Point2i)(ROIStart-BoardLocation+halfsize), (Point2i)(ROIEnd-BoardLocation+halfsize));

	printf("img1 type = %d, img2 type = %d should be %d %d %d %d\n", OverlayRotated.type(), image.type(), CV_8UC4, CV_16UC4, CV_32SC4, CV_32FC4);

	cuda::alphaComp(cuda::GpuMat(OverlayRotated, ROIrobot), cuda::GpuMat(image, ROIImage), cuda::GpuMat(image, ROIImage), cuda::ALPHA_OVER);*/

	cuda::alphaComp(OverlayRotated, image, image, cuda::ALPHA_OVER);
}

cuda::GpuMat& GetRobotImage()
{
	return robot;
}