#include "GlobalConf.hpp"

#include "data/ImageTypes.hpp"

#include <iostream>
#include <libconfig.h++>
#include <X11/Xlib.h> //window resolution

using namespace std;
using namespace cv;
using namespace libconfig;

Ptr<aruco::Dictionary> dict;
Ptr<aruco::DetectorParameters> parameters;
vector<UMat> MarkerImages;

bool ConfigInitialised = false;
Config cfg;

//Default values
CaptureConfig CaptureCfg = {(int)CameraStartType::GSTREAMER_CPU, Size(1920,1080), Rect(0,0,0,0), 1.f, 60, 1, ""};
WebsocketConfig WebsocketCfg = {"eth1", true, true, "127.0.0.1", 42069};
vector<InternalCameraConfig> CamerasInternal;
Size screensize(-1,-1);

template<class T>
Setting& EnsureExistCfg(Setting& Location, const char *FieldName, Setting::Type SettingType, T DefaultValue)
{
	if (!Location.exists(FieldName))
	{
		Setting& settingloc = Location.add(FieldName, SettingType);
		switch (SettingType)
		{
		case Setting::TypeGroup:
		case Setting::TypeArray:
		case Setting::TypeList:
			/* code */
			break;
		
		default:
			settingloc = DefaultValue;
			break;
		}
		return settingloc;
	}
	return Location[FieldName];
}

template<class T>
Setting& CopyDefaultCfg(Setting& Location, const char *FieldName, Setting::Type SettingType, T& DefaultValue)
{
	if (!Location.exists(FieldName))
	{
		Setting& settingloc = Location.add(FieldName, SettingType);
		switch (SettingType)
		{
		case Setting::TypeGroup:
		case Setting::TypeArray:
		case Setting::TypeList:
			/* code */
			break;
		
		default:
			settingloc = DefaultValue;
			break;
		}
		return settingloc;
	}
	else
	{
		DefaultValue = (T)Location[FieldName];
		return Location[FieldName];
	}
}

template<class T>
Setting& CopyDefaultVector(Setting& Location, const char *FieldName, Setting::Type SettingType, vector<T>& DefaultValue)
{
	if(Location.exists(FieldName))
	{
		if (Location[FieldName].isArray())
		{
			Setting& Arrayloc = Location[FieldName];
			DefaultValue.clear();
			for (int i = 0; i < Arrayloc.getLength(); i++)
			{
				DefaultValue.push_back(Arrayloc[i]);
			}
			return Arrayloc;
		}
		else
		{
			Location.remove(FieldName);
		}
		
	}
	Setting& settingloc = Location.add(FieldName, Setting::TypeArray);
	switch (SettingType)
	{
	case Setting::TypeGroup:
	case Setting::TypeArray:
	case Setting::TypeList:
		/* code */
		break;
	
	default:
		for (int i = 0; i < DefaultValue.size(); i++)
		{
			settingloc.add(SettingType) = DefaultValue[i];
		}
		
		break;
	}
	return settingloc;
}

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

	Setting& Capture = EnsureExistCfg(root, "Capture", Setting::Type::TypeGroup, 0);
	
	{
		Setting& Resolution = EnsureExistCfg(Capture, "Resolution", Setting::TypeGroup, 0);
		CopyDefaultCfg(Resolution, "Width", Setting::TypeInt, CaptureCfg.FrameSize.width);
		CopyDefaultCfg(Resolution, "Height", Setting::TypeInt, CaptureCfg.FrameSize.height);
		CopyDefaultCfg(Resolution, "CropLeft", Setting::TypeInt, CaptureCfg.CropRegion.x);
		CopyDefaultCfg(Resolution, "CropTop", Setting::TypeInt, CaptureCfg.CropRegion.y);
		CopyDefaultCfg(Resolution, "CropRight", Setting::TypeInt, CaptureCfg.CropRegion.width);
		CopyDefaultCfg(Resolution, "CropBottom", Setting::TypeInt, CaptureCfg.CropRegion.height);
		CopyDefaultCfg(Capture, "Framerate", Setting::TypeInt, CaptureCfg.CaptureFramerate);
		CopyDefaultCfg(Capture, "FramerateDivider", Setting::TypeInt, CaptureCfg.FramerateDivider);
		CopyDefaultCfg(Capture, "Method", Setting::TypeInt, CaptureCfg.StartType);
		CopyDefaultCfg(Resolution, "Reduction", Setting::TypeFloat, CaptureCfg.ReductionFactor);
		CopyDefaultCfg(Capture, "CameraFilter", Setting::TypeString, CaptureCfg.filter);
	}

	Setting& Websocket = EnsureExistCfg(root, "Websocket", Setting::Type::TypeGroup, 0);
	{
		CopyDefaultCfg(Websocket, "Interface", Setting::TypeString, WebsocketCfg.Interface);
		CopyDefaultCfg(Websocket, "TCP", Setting::TypeBoolean, WebsocketCfg.TCP);
		CopyDefaultCfg(Websocket, "Server", Setting::TypeBoolean, WebsocketCfg.Server);

		CopyDefaultCfg(Websocket, "IP", Setting::TypeString, WebsocketCfg.IP);
		CopyDefaultCfg(Websocket, "Port", Setting::TypeInt, WebsocketCfg.Port);
	}

	Setting& CamerasSett = EnsureExistCfg(root, "InternalCameras", Setting::Type::TypeList, 0);
	{
		CamerasInternal.clear();
		for (int i = 0; i < CamerasSett.getLength(); i++)
		{
			CamerasInternal.push_back(InternalCameraConfig());
			CamerasInternal[i].CameraName = "GarbageFilter";
			CamerasInternal[i].LocationRelative = Affine3d::Identity().translate(Vec3d(0.1,0.2,0.3));
			CopyDefaultCfg(CamerasSett[i], "Filter", Setting::TypeString, CamerasInternal[i].CameraName);
			Setting& Loc = EnsureExistCfg(CamerasSett[i], "Location", Setting::Type::TypeList, 0);

			for (int j = 0; j < 4; j++)
			{
				if (Loc.getLength() <= j)
				{
					Loc.add(Setting::TypeArray);
				}
				while (!Loc[j].isArray())
				{
					Loc.remove(j);
					Loc.add(Setting::TypeArray);
				}
				
				for (int k = 0; k < 4; k++)
				{
					double &address = CamerasInternal[i].LocationRelative.matrix(j,k);
					float value = address;
					if (Loc[j].getLength() <= k)
					{
						Loc[j].add(Setting::TypeFloat) = address;
					}
					else
					{
						address = Loc[j][k];
					}
				}
			}
		}
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
		parameters->cornerRefinementMethod = GetArucoReduction() == GetFrameSize() ? aruco::CORNER_REFINE_CONTOUR : aruco::CORNER_REFINE_NONE;
		parameters->useAruco3Detection = false;
		//parameters->adaptiveThreshWinSizeMin = 5;
		//parameters->adaptiveThreshWinSizeMax = 5;
		//parameters->adaptiveThreshWinSizeStep = 10;
	}
	return parameters;
}

Size GetScreenSize()
{
	#ifdef WITH_X11
	if (screensize == Size(-1,-1))
	{
		Display* d = XOpenDisplay(NULL);
		Screen*  s = DefaultScreenOfDisplay(d);
		screensize = Size(s->width, s->height);
		XCloseDisplay(d);
	}
	return screensize;
	#else
	return Size(1920,1080);
	#endif
}

Size GetFrameSize()
{
	InitConfig();
	return CaptureCfg.FrameSize;
}

int GetCaptureFramerate()
{
	InitConfig();
	return (int)CaptureCfg.CaptureFramerate;
}

CameraStartType GetCaptureMethod()
{
	InitConfig();
	return (CameraStartType)CaptureCfg.StartType;
}

CaptureConfig GetCaptureConfig()
{
	InitConfig();
	return CaptureCfg;
}

float GetReductionFactor()
{
	InitConfig();
	return CaptureCfg.ReductionFactor;
}

Size GetArucoReduction()
{
	Size reduction;
	Size basesize = GetFrameSize();
	float reductionFactor = GetReductionFactor();
	reduction = Size(basesize.width / reductionFactor, basesize.height / reductionFactor);
	return reduction;
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

WebsocketConfig& GetWebsocketConfig()
{
	InitConfig();
	return WebsocketCfg;
}

vector<InternalCameraConfig>& GetInternalCameraPositionsConfig()
{
	InitConfig();
	return CamerasInternal;
}