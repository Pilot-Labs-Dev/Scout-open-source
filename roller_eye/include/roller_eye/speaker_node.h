#pragma once

#include <thread>

#include "roller_eye/decoder_aac.h"

 #include"ros/ros.h"
#include "std_msgs/Int32.h"
#include "roller_eye/frame.h"
#include <alsa/asoundlib.h>
#include <list>
#include <queue>
#include <mutex>
#include "roller_eye/single_class.h"
#include "roller_eye/sound_effects_mgr.h"


#define SPEAKER_NODE_TAG    "SpeakerNode"

#define AAC_BUF_LEN 1024
#define PCM_BUF_LEN (8192*2)
#define MUTE_BUF_LEN 2048

#define PCM_QUE_LEN 5
#define MAX_FRAME_LEN 5

namespace roller_eye
{
    typedef struct pcmData_t{
        int len=0;
        uint8_t pcmBuf[PCM_BUF_LEN];
    }pcmData;
	typedef struct AACData_t{
		int len=0;
		uint8_t Data[PCM_BUF_LEN];
	}AACData;

class SpeakerNode{
    public:
    SpeakerNode();
    ~SpeakerNode();

    void Start();

    bool IsRunning(){return mRunning;}
    int    PlayAudio(uint8_t *buf, int len);

private:
    void Stop();
    void PlayLoop();
    void AAC2PCMLoop();
    int set_hardware_params(uint32_t sample_rate, int channels, int format_size);
    void OnCmd(const std_msgs::Int32::ConstPtr& msg);

    void OnFrame(const roller_eye::frameConstPtr& msg);
    shared_ptr<AACData> GetAACFrame();

    void PutPCM(shared_ptr<pcmData> PCMData);
    shared_ptr<pcmData> GetPCM();

    pcmData* GetPcmBuf(std::queue<pcmData*>& que);
    void PushPcm(pcmData* pcm, std::queue<pcmData*>& que);
    std::vector<uint8_t> GetFrame();

    void ThreadInfo(char *function);

    // std::thread mDecoderThread;
    // std::thread mPlayThread;
    //rmwei add .
    std::shared_ptr<std::thread> m_DecoderThread = nullptr;
    std::shared_ptr<std::thread>  m_PlayThread = nullptr;

    bool mRunning = false;
    mutex mRxMutex;
    mutex mPcmMutex;

    mutex mCMDMutex;

    mutex mPlayLoopMutex;
	mutex mAAC2PCMLoopMutex;

    int mBufsize = 0;
    snd_pcm_t *mHandle=nullptr;
    snd_pcm_hw_params_t *mParams=nullptr;

    snd_pcm_uframes_t mFrames;
   //ros::NodeHandle mGlobal;
   ros::Subscriber mCmdSub;
   ros::Subscriber mFrameSub;

   std::queue< shared_ptr<AACData> > mAACQue;
   std::queue< shared_ptr<pcmData> > mPCMQue;

   std::queue<std::vector<uint8_t>> mframeQue;
   std::queue<std::vector<uint8_t>> mlvPcm;

   std::queue<pcmData*> mpcmFreeQue;

   std::queue<pcmData*> mpcmQue;
   shared_ptr<ros::NodeHandle> mGlobal;
   shared_ptr<SoundEffectsMgr>  mSEM;
};
} // namespace roller_eye
