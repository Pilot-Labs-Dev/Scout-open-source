#pragma once

#include <thread>

#include"roller_eye/single_class.h"
#include "roller_eye/decoder_aac.h"

#include <alsa/asoundlib.h>

 #include"ros/ros.h"

#define PCM_BUF_LEN 8192
namespace roller_eye
{
class SpeakerMgr:public SingleClass<SpeakerMgr>{
        friend class SingleClass<SpeakerMgr>;
    
    SpeakerMgr();
    public:

    ~SpeakerMgr();

    void Start(int avIdx);
    void Stop(int avIdx);
    void Play(uint8_t *buf, int len);

    bool IsRunning(){return mRunning;}

private:
    void Stop();
    void PublishData(uint8_t* data, int len);
    void RecvSpeechAudioLoop();

    int  mAvIdx;
    bool mRunning = false;
    std::thread mRxThread;
    timed_mutex mMutex;
    ros::NodeHandle mGlobal;
    ros::Publisher mFramePub;
    ros::Publisher mCmdPub;
};
} // namespace roller_eye
