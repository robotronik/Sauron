#pragma once

#include <opencv2/core.hpp>
#include "position.hpp"

using namespace cv;

//Charge les images (plateau, robot, palet...) dans la mémoire GPU
void InitImages(); 

//initialise le terrain de jeu (taille et centre du terrain)
void CreateSpace(FVector2D<float> extent, FVector2D<float> center);

//Regénère l'image de fond
void CreateBackground(Size Resolution);

//Recupère l'image finale
void GetImage(UMat& rvalue);

//Ajoute une image par dessus le reste, le centre de l'image se trouvant à Position dans le terrain
void OverlayImage(cuda::GpuMat& ImageToOverlay, FVector2D<float> position, float rotation, FVector2D<float> ImageSize);

//fonction de test captivante (litérallement et figurativement)
void TestBoardViz();

cuda::GpuMat& GetRobotImage();