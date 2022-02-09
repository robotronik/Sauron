#pragma once

#include <opencv2/core.hpp>
#include <opencv2/core/cuda.hpp>
#include "position.hpp"
#include "OutputImage.hpp"

using namespace cv;

enum class PaletCouleur
{
	autre=17,
	rouge=47,
	vert=36,
	bleu=13
};

class boardviz : public OutputImage
{
private:
	static bool ImagesLoaded;
	static cuda::GpuMat table;
	static cuda::GpuMat robot;
	static cuda::GpuMat rouge, vert, bleu, autre;
	static cuda::GpuMat camera;

private:
	FVector2D<float> Extent;
	FVector2D<float> Center;
	cuda::GpuMat image;


public:
	boardviz(FVector2D<float> InExtent, FVector2D<float> InCenter)
		:Extent(InExtent),
		Center(InCenter)
	{}
	~boardviz()
	{}

private:

	static void LoadImage(cuda::GpuMat& output, String location);
	FVector2D<int> BoardToPixel(FVector2D<float> location);
	FVector2D<float> GetImageCenter(cuda::GpuMat& Image);

public:

	//Charge les images (plateau, robot, palet...) dans la mémoire GPU
	static void InitImages();

	//Regénère l'image de fond
	void CreateBackground(Size Resolution);

	//Recupère l'image finale
	void GetImage(UMat& rvalue);

	//Ajoute une image par dessus le reste, le centre de l'image se trouvant à Position dans le terrain
	void OverlayImage(cuda::GpuMat& ImageToOverlay, FVector2D<float> position, float rotation, FVector2D<float> ImageSize);

	FVector2D<float> GetCenter();
	FVector2D<float> GetExtent();

	static cuda::GpuMat& GetRobotImage();
	static cuda::GpuMat& GetPalet(PaletCouleur type);
	static cuda::GpuMat& GetCamera();

	virtual void GetOutputFrame(int BufferIndex, UMat& frame, Size winsize) override;
};


//fonction de test captivante (litérallement et figurativement)
void TestBoardViz();



