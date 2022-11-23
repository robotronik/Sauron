#include "visualisation/BoardViz2D.hpp"

#include <math.h>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/viz.hpp>

#ifdef WITH_CUDA
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudawarping.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudaimgproc.hpp>
#else
#include <opencv2/imgproc.hpp>
#endif

#include "data/FrameCounter.hpp"
#include "math3d.hpp"

using namespace cv;


const String assetpath = "../assets/";

bool BoardViz2D::ImagesLoaded;
BoardImageType BoardViz2D::table;
BoardImageType BoardViz2D::robot;
BoardImageType BoardViz2D::rouge, BoardViz2D::vert, BoardViz2D::bleu, BoardViz2D::autre;
BoardImageType BoardViz2D::camera;

void BoardViz2D::LoadImage(BoardImageType& output, String location)
{
	#ifdef WITH_CUDA
	output.upload(imread(assetpath + location, IMREAD_UNCHANGED));
	#else
	Mat temp = imread(assetpath + location, IMREAD_UNCHANGED);
	temp.copyTo(output);
	#endif
}

FVector2D<int> BoardViz2D::BoardToPixel(FVector2D<float> location)
{
	return (location+Center)/Extent*FVector2D<int>(image.size());
}

FVector2D<float> BoardViz2D::GetImageCenter(BoardImageType& Image)
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
	#ifdef WITH_CUDA
	cuda::resize(table, image, Resolution);
	#else
	resize(table, image, Resolution);
	#endif
}

void BoardViz2D::GetImage(UMat& rvalue)
{
	#ifdef WITH_CUDA
	image.download(rvalue);
	#else
	rvalue = image;
	#endif
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

void BoardViz2D::OverlayImage(BoardImageType& ImageToOverlay, FVector2D<float> position, float rotation, FVector2D<float> ImageSize)
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
	BoardImageType OverlayRotated(Oversize.ToSize(), ImageToOverlay.type());

	Point2f dstTri[3];
	dstTri[0] = GetImageCenter(OverlayRotated);
	dstTri[1] = RotatePointAround(dstTri[0], FVector2D<float>(dstTri[0]) - FVector2D<float>(TargetSize.x,0), rotation);
	dstTri[2] = RotatePointAround(dstTri[0], FVector2D<float>(dstTri[0]) - FVector2D<float>(0,TargetSize.y), rotation);
	Mat warp_mat = getAffineTransform( srcTri, dstTri );

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

	#ifdef WITH_CUDA
	cuda::warpAffine(ImageToOverlay, OverlayRotated, warp_mat, OverlayRotated.size());
	cuda::alphaComp(BoardImageType(OverlayRotated, ROIrobot), BoardImageType(image, ROIImage), BoardImageType(image, ROIImage), cuda::ALPHA_OVER);
	#else
	warpAffine(ImageToOverlay, BoardImageType(image, ROIImage), warp_mat, OverlayRotated.size(), 1, BORDER_TRANSPARENT);
	#endif
	

	//printf("img1 type = %d, img2 type = %d should be %d %d %d %d\n", OverlayRotated.type(), image.type(), CV_8UC4, CV_16UC4, CV_32SC4, CV_32FC4);
	//cuda::alphaComp(OverlayRotated, image, image, cuda::ALPHA_OVER);
}

void BoardViz2D::OverlayImage(BoardImageType& ImageToOverlay, Affine3d position, FVector2D<float> ImageSize)
{
	Vec3d translation3d = position.translation();
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

BoardImageType& BoardViz2D::GetRobotImage()
{
	return robot;
}

BoardImageType& BoardViz2D::GetPalet(PaletCouleur type)
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

BoardImageType& BoardViz2D::GetCamera()
{
	return camera;
}

void BoardViz2D::DisplayData(std::vector<ObjectData> &objects)
{
	InitImages();
	Size matsize(1500, 1000);
	#ifdef WITH_CUDA
	image = cuda::createContinuous(1500, 1000, CV_8UC4);
	#else
	image = UMat(matsize, CV_8UC3, Scalar(0,0,255));
	#endif
	for (int i = 0; i < objects.size(); i++)
	{
		ObjectData &object = objects[i];
		switch (object.identity.type)
		{
		case PacketType::Camera :
			OverlayImage(camera, object.location, FVector2D(0.2,0.2));
			break;
		case PacketType::ReferenceAbsolute :
			CreateBackground(Size(1500, 1000));
			break;
		case PacketType::ReferenceRelative :
			OverlayImage(table, object.location, FVector2D(3.0,2.0));
			break;
		case PacketType::Robot :
			OverlayImage(robot, object.location, FVector2D(0.25,0.25));
			break;
		case PacketType::Puck :
			break;
		
		default:
			break;
		}
	}
	
}

void BoardViz2D::GetOutputFrame(int BufferIndex, UMat& OutFrame, Rect window)
{
	BoardImageType resizedgpu, recoloredgpu;

	Size insize = window.size();
	Size bssize = image.size();
	
	Rect roi = ScaleToFit(bssize, window);

	#ifdef WITH_CUDA
	cuda::resize(image, resizedgpu, roi.size(), 0, 0, INTER_LINEAR);
	cuda::cvtColor(resizedgpu, recoloredgpu, COLOR_BGRA2BGR);
	recoloredgpu.download(OutFrame(roi));
	#else
	UMat resized;
	resize(image, resized, roi.size(), 0, 0, INTER_LINEAR);
	cvtColor(resized, OutFrame(roi), COLOR_BGRA2BGR);
	#endif
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