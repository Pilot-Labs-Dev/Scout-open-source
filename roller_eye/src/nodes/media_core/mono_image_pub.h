#ifndef __ROLLER_EYE_MONO_IMG_PUB__H__
#define __ROLLER_EYE_MONO_IMG_PUB__H__
#include"ros/ros.h"
#include"sensor_msgs/Image.h"
#include"roller_eye/video_stream.h"
#include"roller_eye/plt_tools.h"
#include "roller_eye/camera_handle.hpp"

#define MONO_PUB_SUGEST_W                   640
#define MONO_PUB_SUGEST_H                    480

// #define MONO_PUB_SUGEST_W                   1080
// #define MONO_PUB_SUGEST_H                    720
#define MONO_PUB_FPS                                   10

namespace roller_eye
{
class MonoImgPub{
public:
    MonoImgPub();
    ~MonoImgPub();
    void start();
    void stop();
    int setData(sensor_msgs::Image& s);
    void onParamChanged(CaputreParam *param, bool resume);
private:
    void freeVideoStream();
    void getVideoStream();
    /*
    计算mono时间和epoch时间的差值
    */
    int64_t getEpochMonoShift();//unit us
    VSHandle mVideoStream;
    int mFilter;
    int mSkipCnt;
    bool mFirstFrame;
    struct timeval mLastTimeVal;
    int64_t mLastEpchoShift;
    int mEpchoFreshCnt;
    bool mNeedResume;
};
} // namespace roller_eye
#endif