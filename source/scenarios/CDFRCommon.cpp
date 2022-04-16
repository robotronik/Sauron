#include "Scenarios/CDFRCommon.hpp"


void BufferedPipeline(int BufferCaptureIdx, vector<Camera*> Cameras, Ptr<aruco::Dictionary> dict, Ptr<aruco::DetectorParameters> params, ObjectTracker* registry)
{
	int numCams = Cameras.size();
	for (int i = 0; i < Cameras.size(); i++)
	{
		Cameras[i]->Grab(BufferCaptureIdx);
	}
	//Range range(0, numCams*Camera::FrameBufferSize);
	parallel_for_(Range(0, numCams * Camera::FrameBufferSize), [&](const Range& range)
	{
		for (int i = range.start; i < range.end; i++)
		{
			int BufferIdx = (BufferCaptureIdx + i/numCams) % Camera::FrameBufferSize;
			int CamIdx = i%numCams;
			switch (i/numCams)
			{
			case 0:
				// read
				
				
				break;
			
			case 1:
				//Detect aruco
				Cameras[CamIdx]->Read(BufferIdx);
				Cameras[CamIdx]->RescaleFrames(BufferIdx);
				Cameras[CamIdx]->detectMarkers(BufferIdx, dict, params);
				Cameras[CamIdx]->SolveMarkers(BufferIdx, CamIdx, registry);
				break;

			default:
				cout << "Frame buffer too big or operation unimplemented" << endl;
				break;
			}
			
			
		}
		//cout << "Aruco stripe from " << range.start << " to " << range.end << endl;
	}, numCams*Camera::FrameBufferSize);
} 
