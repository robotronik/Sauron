#pragma once

#include <opencv2/core.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/core/affine.hpp>
#include "data/FVector2D.hpp"
#include "data/OutputImage.hpp"

enum class PaletCouleur
{
	autre=17,
	rouge=47,
	vert=36,
	bleu=13
};

class BoardViz2D : public OutputImage
{
private:
	static bool ImagesLoaded;
	static cv::cuda::GpuMat table;
	static cv::cuda::GpuMat robot;
	static cv::cuda::GpuMat rouge, vert, bleu, autre;
	static cv::cuda::GpuMat camera;

private:
	FVector2D<float> Extent;
	FVector2D<float> Center;
	cv::cuda::GpuMat image;


public:
	BoardViz2D(FVector2D<float> InExtent, FVector2D<float> InCenter)
		:Extent(InExtent),
		Center(InCenter)
	{}
	~BoardViz2D()
	{}

private:

	static void LoadImage(cv::cuda::GpuMat& output, cv::String location);
	FVector2D<int> BoardToPixel(FVector2D<float> location);
	FVector2D<float> GetImageCenter(cv::cuda::GpuMat& Image);

public:

	//Charge les images (plateau, robot, palet...) dans la mémoire GPU
	static void InitImages();

	//Regénère l'image de fond
	void CreateBackground(cv::Size Resolution);

	//Recupère l'image finale
	void GetImage(cv::UMat& rvalue);

	//Ajoute une image par dessus le reste, le centre de l'image se trouvant à Position dans le terrain
	void OverlayImage(cv::cuda::GpuMat& ImageToOverlay, FVector2D<float> position, float rotation, FVector2D<float> ImageSize);

	//Ajoute une image, assume que X+ correspond au X+ de l'image, Y+ correspond au Y+ de l'image, et Z correspond à la rotation
	//Pour une image, le X+ va à droite et le Y+ va vers le bas
	void OverlayImage(cv::cuda::GpuMat& ImageToOverlay, cv::Affine3d position, FVector2D<float> ImageSize);

	FVector2D<float> GetCenter();
	FVector2D<float> GetExtent();

	static cv::cuda::GpuMat& GetRobotImage();
	static cv::cuda::GpuMat& GetPalet(PaletCouleur type);
	static cv::cuda::GpuMat& GetCamera();

	virtual void GetOutputFrame(int BufferIndex, cv::UMat& frame, cv::Size winsize) override;
};


//fonction de test captivante (litérallement et figurativement)
void TestBoardViz();



