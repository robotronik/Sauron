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
		parameters->cornerRefinementMethod = GetArucoReductions()[0] == GetFrameSize() ? aruco::CORNER_REFINE_CONTOUR : aruco::CORNER_REFINE_NONE;
		//parameters->adaptiveThreshWinSizeMin = 5;
		//parameters->adaptiveThreshWinSizeMax = 5;
		//parameters->adaptiveThreshWinSizeStep = 10;
	}
	return parameters;
}

Size GetScreenSize()
{
	return Size(1920,1080);
}

Size GetFrameSize()
{
	//return Size(720,480);
	return Size(1920,1080);
}

int GetCaptureFramerate()
{
	//return 30;
	return 60;
}

vector<float> GetReductionFactor()
{
	return {2};
}

vector<Size> GetArucoReductions()
{
	vector<Size> reductions;
	Size basesize = GetFrameSize();
	vector<float> reductionFactors  = GetReductionFactor();
	reductions.resize(reductionFactors.size());
	for (int i = 0; i < reductionFactors.size(); i++)
	{
		reductions[i] = Size(basesize.width / reductionFactors[i], basesize.height / reductionFactors[i]);
	}
	return reductions;
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