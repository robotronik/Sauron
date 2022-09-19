#include "GlobalConf.hpp"

#include <iostream>
#include <libconfig.h++>
#include "data/ImageTypes.hpp"

#include <X11/Xlib.h> //window resolution

using namespace std;
using namespace cv;
using namespace libconfig;

Ptr<aruco::Dictionary> dict;
Ptr<aruco::DetectorParameters> parameters;
vector<UMat> MarkerImages;

bool ConfigInitialised = false;
Config cfg;

void InitConfig()
{
	if (ConfigInitialised)
	{
		return;
	}
	
	bool err = false;
	try
	{
		cfg.readFile("../config.cfg");
	}
		catch(const FileIOException &fioex)
	{
		std::cerr << "I/O error while reading file." << std::endl;
		err = true;
	}
		catch(const ParseException &pex)
	{
		std::cerr << "Parse error at " << pex.getFile() << ":" << pex.getLine()
			<< " - " << pex.getError() << std::endl;
		err = true;
	}

	Setting& root = cfg.getRoot();

	if (!root.exists("CaptureResolution"))
	{
		root.add("CaptureResolution", Setting::Type::TypeArray);
		root["CaptureResolution"].add(Setting::Type::TypeInt) = 1280;
		root["CaptureResolution"].add(Setting::Type::TypeInt) = 720;
	}
	if (!root.exists("CaptureFramerate"))
	{
		root.add("CaptureFramerate", Setting::Type::TypeInt) = 120;
	}
	if (!root.exists("CaptureMethod"))
	{
		root.add("CaptureMethod", Setting::Type::TypeInt) = (int)CameraStartType::GSTREAMER_NVARGUS;
	}
	
	

	cfg.writeFile("../config.cfg");
	
	ConfigInitialised = true;
}

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
	#ifdef WITH_X11
	Display* d = XOpenDisplay(NULL);
	Screen*  s = DefaultScreenOfDisplay(d);
	
	return Size(s->width, s->height);
	#else
	return Size(1920,1080);
	#endif
}

Size GetFrameSize()
{
	InitConfig();
	Setting& resolution = cfg.getRoot()["CaptureResolution"];
	return Size((int)resolution[0], (int)resolution[1]);
}

int GetCaptureFramerate()
{
	InitConfig();
	return (int)(cfg.getRoot()["CaptureFramerate"]);
}

CameraStartType GetCaptureMethod()
{
	InitConfig();
	return (CameraStartType)(int)(cfg.getRoot()["CaptureMethod"]);
}

vector<float> GetReductionFactor()
{
	return {1};
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