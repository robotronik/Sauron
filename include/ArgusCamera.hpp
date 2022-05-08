#pragma once

#include <iostream>
#include <string>   // for strings
#include <opencv2/core.hpp>     // Basic OpenCV structures (Mat, Scalar)
#include <opencv2/highgui.hpp>  // OpenCV window I/O
#include <opencv2/aruco.hpp>

#include <opencv2/cudacodec.hpp>
#include <opencv2/cudawarping.hpp>

#include "thirdparty/list-devices.hpp"

#include "Camera.hpp"

using namespace std;
using namespace cv;


#ifdef WITH_ARGUS
#include <Argus/Argus.h>
#include <EGLStream/EGLStream.h>
#include <Argus/BufferStream.h>

using namespace EGLStream;
using namespace Argus;
#endif
class ArgusCamera : public Camera
{
#ifdef WITH_ARGUS
    unsigned char *m_OutputBuffer;
	int m_dmabuf;
	UniqueObj<FrameConsumer> m_consumer;

	IBufferOutputStream *iEglOutputStream;
	IFrameConsumer *iFrameConsumer;
#endif
public:
	ArgusCamera(CameraSettings InSettings);

	~ArgusCamera();
#ifdef WITH_ARGUS
	virtual bool StartFeed() override;

	virtual bool Grab(int BufferIndex) override;
	
	virtual bool Read(int BufferIndex) override;
#endif
};

#ifdef WITH_ARGUS
int ArgusEGLImage(int argc, char** argv);
#endif