#pragma once

//#include "decoder.h"
#include <faac.h>
#include <thread>
#include"roller_eye/single_class.h"

#include <pulse/simple.h>
#include <pulse/error.h>
#include <pulse/gccmacro.h>

namespace roller_eye
{
class NetSpeaker:public SingleClass<NetSpeaker>{
        friend class SingleClass<NetSpeaker>;
    
    NetSpeaker();
    public:

    ~NetSpeaker();

    public: class AudioIn{
        public:
            int SID;
            int ch;
            int avIndex;
            unsigned long srvType;
            int dspFd;
    };

    void Start(const AudioIn& audioIn);
    void Stop();

    bool IsRunning(){return mRunning;}

    int    playAudio(uint8_t *buf, int len);

private:
    void RecvSpeechAudioLoop();

    std::thread mRxThread;
    bool mRunning = false;
    AudioIn mAudioIn;

    pa_simple *m_ps = NULL;

    mutex mMutex;
};
} // namespace roller_eye
