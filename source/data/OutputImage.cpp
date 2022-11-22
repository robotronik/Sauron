#include "data/OutputImage.hpp"
#include "GlobalConf.hpp"

using namespace cv;
using namespace std;

OutputImage::OutputImage(/* args */)
{
}

OutputImage::~OutputImage()
{
}

void OutputImage::SetFrame(int BufferIndex, UMat& InFrame)
{

}

void OutputImage::GetFrame(int BufferIndex, UMat& OutFrame)
{

}


void OutputImage::GetOutputFrame(int BufferIndex, UMat& OutFrame, Rect window)
{

}

Size findSplit(Size screensize, Size targetAspect, int numscreen)
{
	Size splits(1,1);
	double AspectRatioTarget = (double)targetAspect.width/targetAspect.height;
	double AspectRatioScreen = (double)screensize.width/screensize.height;
	while (splits.height * splits.width < numscreen)
	{
		double arc = AspectRatioScreen/(splits.width+1.0)*splits.height;
		double arr = AspectRatioScreen/splits.width*(splits.height+1.0);
		if (abs(arc - AspectRatioTarget) < abs(arr - AspectRatioTarget))
		{
			splits.width++;
		}
		else
		{
			splits.height++;
		}
	}
	return splits;
}

UMat ConcatCameras(int BufferIndex, vector<OutputImage*> Cameras, int NumCams)
{
	Size screensize = GetScreenSize();
	UMat concatenated(screensize, CV_8UC3, Scalar(0,0,255));
	Size splits = findSplit(screensize, GetFrameSize(), Cameras.size());
	int &rows = splits.height, &columns = splits.width;
	int winWidth = screensize.width/columns, winHeight = screensize.height/rows;
	/*parallel_for_(Range(0, Cameras.size()), [&](const Range& range)
	{*/
	Range range(0, Cameras.size());
		for (int i = range.start; i < range.end; i++)
		{
			Rect roi(winWidth * (i%columns), winHeight * (i / columns), winWidth, winHeight);
			/*UMat frame; Cameras[i]->GetFrame(BufferIndex, frame);
			if (frame.empty())
			{
				continue;
			}*/
			Cameras[i]->GetOutputFrame(0, concatenated, roi);
			//Size offset = (Size(winWidth, winHeight) - region.size())/2;
			//region.copyTo(concatenated(Rect(roi.x+offset.width, roi.y+offset.height, region.cols, region.rows)));
			//region.copyTo(concatenated(roi));
		}
	/*});*/
	return concatenated;
}