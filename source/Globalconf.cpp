#include "GlobalConf.hpp"

using namespace std;
using namespace cv;

Ptr<aruco::Dictionary> dict;
Ptr<aruco::DetectorParameters> parameters;
vector<UMat> MarkerImages;

Ptr<aruco::Dictionary> GetArucoDict(){
	if (dict.empty())
	{
		dict = aruco::getPredefinedDictionary(aruco::DICT_4X4_100);
	}
	return dict;
}

Ptr<aruco::DetectorParameters> GetArucoParams()
{
	if (parameters.empty())
	{
		parameters = aruco::DetectorParameters::create();
		parameters->cornerRefinementMethod = aruco::CORNER_REFINE_CONTOUR;
	}
	return parameters;
}

UMat& GetArucoImage(int id)
{
	if (MarkerImages.size() != 100)
	{
		MarkerImages.resize(100);
	}
	if (MarkerImages[id].empty())
	{
		aruco::drawMarker(GetArucoDict(), id, 256, MarkerImages[id], 1);
	}
	return MarkerImages[id];
}