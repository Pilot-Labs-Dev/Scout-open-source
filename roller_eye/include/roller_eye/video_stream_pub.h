#ifndef __VIDEO_STREAM_PUB_H__
#define __VIDEO_STREAM_PUB_H__


#include <ros/ros.h>
#include <atomic>
#include <thread>
#include "roller_eye/system_define.h"
#include "roller_eye/plterrno.h"
#include "roller_eye/camera_handle.hpp"
#include "roller_eye/encoder.h"
#include "roller_eye/frame.h"
#include "roller_eye/detect.h"
#include "roller_eye/publisher.h"

namespace roller_eye {

#if 0
#define V_STREAM_VERBOSE(fmt, ...)  printf(fmt, ##__VA_ARGS__)
#else
#define V_STREAM_VERBOSE(fmt, ...)  (void)0
#endif

const std::string BLEND_FILE_YUYV = string(ROLLER_EYE_FILE_HOME_PATH) + "moorebot_wm160x30_yuyv422.yuv";
const std::string BLEND_FILE_YUV420P = string(ROLLER_EYE_FILE_HOME_PATH) + "moorebot_wm160x30_yuv420p.yuv";
const int ALPHA_VAL = 30; //[0,255]
const int BLEND_WIDTH = 160;
const int BLEND_HEIGHT = 30;

struct blend_frame_t
{
	uint8_t *data = nullptr;
	int width = BLEND_WIDTH;
	int height = BLEND_HEIGHT;
};

class VideoStreamPub
{
public:
	VideoStreamPub(ros::NodeHandle& nh, CameraHandle *ch);
	 ~VideoStreamPub();

    void subConnectH264(int cnt);
    void subDisconnectH264(int cnt);
	void objCallback(const detect& msg);

	void startPublish();
	void stopPublish();

	CaputreParam getCaptureParam(void);
	int getNumSubscribers();

private:
	VideoStreamPub(const VideoStreamPub &);
	VideoStreamPub& operator=(const VideoStreamPub &);

	FrameBuff *getFrame();
	void initBldFrame(int fmt);
	void onParamChanged(CaputreParam *param, bool resume);
	void publishLoop();
	// void publishRaw(Frame *tmpFrame);
	void publishH264(Frame *tmpFrame);

    int subRefNum;
	std::atomic<bool> m_pubRunning;
	std::thread m_pubThread;

    CameraHandle *mCameraHandle;
	VSHandle vsHandle;
	uint32_t mSessionH264;

	std::shared_ptr<Encoder> mEncoder;
    Publisher<roller_eye::frame> mPubH264;
	detect mObjDet;
	blend_frame_t mBldFrame;
	bool mNeedResume;
};

} // namespace roller_eye

#endif // __VIDEO_STREAM_PUB_H__

