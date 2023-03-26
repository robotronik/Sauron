#include "Scenarios/CDFRCommon.hpp"
#include "data/ManualProfiler.hpp"

ManualProfiler mp("Pipeline ", {"Read", "Undistort", "Rescale", "ArucoDetect", "ArucoSolve"});

void BufferedPipeline(int BufferCaptureIdx, vector<ArucoCamera*> Cameras, aruco::ArucoDetector& Detector, ObjectTracker* registry)
{
	int numCams = Cameras.size();
	vector<uint8_t> BufToCamMap;
	vector<uint8_t> BufIdxMap;
	for (int i = 0; i < Cameras.size(); i++)
	{
		Cameras[i]->Grab(BufferCaptureIdx);
		for (size_t j = 0; j < 1/*Cameras[i]->GetCameraSettings().BufferSize*/; j++)
		{
			BufToCamMap.push_back(i);
			BufIdxMap.push_back(j);
		}
		
	}
	//Range range(0, numCams*Camera::FrameBufferSize);
	vector<ManualProfiler> mps;
	mps.resize(BufToCamMap.size());
	parallel_for_(Range(0, BufToCamMap.size()), [&](const Range& range)
	{
		for (int i = range.start; i < range.end; i++)
		{
			ManualProfiler& localprof = mps[i];
			int pf = 0;
			localprof.EnterSection(pf++);
			int CamIdx = BufToCamMap[i];
			ArucoCamera* cam = Cameras[CamIdx];
			int Buffer0 = BufIdxMap[i];
			int BufferIdx = (BufferCaptureIdx + Buffer0) % cam->GetCameraSettings().BufferSize;
			//Detect aruco
			cam->Read(BufferIdx);
			localprof.EnterSection(pf++);
			cam->Undistort(BufferIdx);
			localprof.EnterSection(pf++);
			cam->RescaleFrames(BufferIdx);
			localprof.EnterSection(pf++);
			cam->detectMarkers(BufferIdx, Detector);
			//localprof.EnterSection(pf++);
			//cam->SolveMarkers(BufferIdx, CamIdx, registry);
			localprof.EnterSection(-1);
			
			
		}
		//cout << "Aruco stripe from " << range.start << " to " << range.end << endl;
	}, BufToCamMap.size());
	/*for (int i = 0; i < BufToCamMap.size(); i++)
	{
		mp += mps[i];
	}*/
	//mp.PrintIfShould();
	
} 

unordered_map<PacketType, bool> GetDefaultAllowMap()
{
	unordered_map<PacketType, bool> allowmap = {
		{PacketType::Null, false},
		{PacketType::Camera, false},
		{PacketType::ReferenceAbsolute, true},
		{PacketType::ReferenceRelative, true},
		{PacketType::Robot, true},
		{PacketType::TrackerCube, true},
		{PacketType::TopTracker, true},
		{PacketType::Puck, true},
		{PacketType::Tag, false}
	};
	return allowmap;
}