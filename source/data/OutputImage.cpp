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

void OutputImage::GetOutputFrame(int BufferIndex, UMat& OutFrame, Size winsize)
{

}

UMat ConcatCameras(int BufferIndex, vector<OutputImage*> Cameras, int NumCams)
{
	Size screensize = GetScreenSize();
	UMat concatenated(screensize, CV_8UC3, Scalar(0,0,255));
	int rows = 1, columns = 1;
	while (rows * rows < NumCams)
	{
		rows++;
	}
	columns = rows;
	int winWidth = screensize.width/rows, winHeight = screensize.height/columns;
	parallel_for_(Range(0, Cameras.size()), [&](const Range& range)
	{
		for (int i = range.start; i < range.end; i++)
		{
			Rect roi(winWidth * (i%rows), winHeight * (i / rows), winWidth, winHeight);
			/*UMat frame; Cameras[i]->GetFrame(BufferIndex, frame);
			if (frame.empty())
			{
				continue;
			}*/
			UMat region = concatenated(roi);
			Cameras[i]->GetOutputFrame(0, region, Size(winWidth, winHeight));
		}
	});
	return concatenated;
}