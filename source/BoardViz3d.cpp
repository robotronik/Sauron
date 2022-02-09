#include "BoardViz3d.hpp"
#include <opencv2/viz.hpp>

#include "GlobalConf.hpp"
#include "trackedobject.hpp"

const String assetpath = "../assets/";

BoardViz3D::BoardViz3D()
{
}

BoardViz3D::~BoardViz3D()
{
}




void TestViz3D()
{
	viz::Viz3d board("3D Board") ;
	board.showWidget("axis", viz::WCoordinateSystem(0.1));
	viz::Mesh mesh = viz::Mesh::load(assetpath + "board.obj", viz::Mesh::LOAD_OBJ);
	board.showWidget("mesh", viz::WMesh(mesh));
    Mat boardimg = imread(assetpath + "VisuelTable.png");
    viz::WImage3D tableImg(boardimg, Size2d(3, 2), Vec3d(0,-1e-3, 0), Vec3d(0,-1,0), Vec3d(0,0,1));
    board.showWidget("table", tableImg);
    UMat arucoimg = GetArucoImage(center.number);
    viz::WImage3D centerImg(arucoimg, Size2d(center.sideLength, center.sideLength), center.OffsetLocation, Vec3d(0,-1,0), Vec3d(0,0,1));
    board.showWidget("center", centerImg);
	board.spin();
	exit(EXIT_SUCCESS);
}