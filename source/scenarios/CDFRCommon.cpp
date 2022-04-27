#include "Scenarios/CDFRCommon.hpp"


void BufferedPipeline(int BufferCaptureIdx, vector<Camera*> Cameras, Ptr<aruco::Dictionary> dict, Ptr<aruco::DetectorParameters> params, ObjectTracker* registry)
{
	int numCams = Cameras.size();
	vector<uint8_t> BufToCamMap;
	vector<uint8_t> BufIdxMap;
	for (int i = 0; i < Cameras.size(); i++)
	{
		Cameras[i]->Grab(BufferCaptureIdx);
		for (size_t j = 0; j < Cameras[i]->GetCameraSettings().BufferSize; j++)
		{
			BufToCamMap.push_back(i);
			BufIdxMap.push_back(j);
		}
		
	}
	//Range range(0, numCams*Camera::FrameBufferSize);
	parallel_for_(Range(0, BufToCamMap.size()), [&](const Range& range)
	{
		for (int i = range.start; i < range.end; i++)
		{
			int CamIdx = BufToCamMap[i];
			Camera* cam = Cameras[CamIdx];
			int Buffer0 = BufIdxMap[i];
			int BufferIdx = (BufferCaptureIdx + Buffer0) % cam->GetCameraSettings().BufferSize;
			switch (i/numCams)
			{
			case 0:
				// read
				
				
				break;
			
			case 1:
				//Detect aruco
				cam->Read(BufferIdx);
				cam->Undistort(BufferIdx);
				cam->RescaleFrames(BufferIdx);
				cam->detectMarkers(BufferIdx, dict, params);
				cam->SolveMarkers(BufferIdx, CamIdx, registry);
				break;

			default:
				cout << "Frame buffer too big or operation unimplemented" << endl;
				break;
			}
			
			
		}
		//cout << "Aruco stripe from " << range.start << " to " << range.end << endl;
	}, BufToCamMap.size());
} 
