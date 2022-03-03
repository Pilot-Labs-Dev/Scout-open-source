#ifndef __PIC_STREAM_PUB_H__
#define __PIC_STREAM_PUB_H__


#include <ros/ros.h>
#include <atomic>
#include <thread>
#include "roller_eye/plterrno.h"
#include "roller_eye/camera_handle.hpp"
#include "roller_eye/encoder.h"
#include "roller_eye/frame.h"
#include "roller_eye/publisher.h"

namespace roller_eye {

#if 0
#define PIC_STREAM_VERBOSE(fmt, ...)  printf(fmt, ##__VA_ARGS__)
#else
#define PIC_STREAM_VERBOSE(fmt, ...)  (void)0
#endif

class PicStreamPub
{
public:
	PicStreamPub(ros::NodeHandle& nh, CameraHandle *ch);
	 ~PicStreamPub();

    void subConnectJPG(int cnt);
    void subDisconnectJPG(int cnt);

	void startPublish();
	void stopPublish();

	CaputreParam getCaptureParam(void);
	int getNumSubscribers();

private:
	PicStreamPub(const PicStreamPub &);
	PicStreamPub& operator=(const PicStreamPub &);

	FrameBuff *getFrame();
	void onParamChanged(CaputreParam *param, bool resume);
	void publishLoop();
	void publishJPG(Frame *tmpFrame);

    int subRefNum;
	std::atomic<bool> m_pubRunning;
	std::thread m_pubThread;

    CameraHandle *mCameraHandle;
	VSHandle vsHandle;
	uint32_t mSessionJPG;

	std::unique_ptr<Encoder> mEncoder;
    Publisher<roller_eye::frame> mPubJPG;
	bool mNeedResume;
};

} // namespace roller_eye

#endif // __PIC_STREAM_PUB_H__

