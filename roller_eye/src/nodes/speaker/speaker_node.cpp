#define ALSA_PCM_NEW_HW_PARAMS_API

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/wait.h>
#include <alsa/asoundlib.h>
#include <sys/ioctl.h>
#include <sys/syscall.h>
#include <sys/prctl.h>

#include "roller_eye/speaker_node.h"

#define u32 unsigned int
#define u8   unsigned char
#define u16 unsigned short

#include <string.h>
#include "roller_eye/plog_plus.h"
#include "roller_eye/frame.h"
#include "roller_eye/system_define.h"
#include<tutk/P2PCam/AVIOCTRLDEFs.h>
#include<tutk/P2PCam/AVFRAMEINFO.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
extern "C" {
    #include "libavformat/avformat.h"
    #include "libavcodec/avcodec.h"
    #include "libswresample/swresample.h"
    #include "libavutil/avutil.h"
}
#include "roller_eye/system_define.h"
#include "roller_eye/aplay.h"
extern "C" {
	bool CanPlayTalk();
	int  InitPlayTalk ();
	void DeinitPlayTalk ();
	int PlayTalkback (char* buffer, unsigned int dataLen);
}
namespace roller_eye
{
SpeakerNode::SpeakerNode()
{
	avcodec_register_all();
    av_register_all();
    mGlobal=make_shared<ros::NodeHandle>("");
    mCmdSub = mGlobal->subscribe("/speaker_cmd", 1, &SpeakerNode::OnCmd,this);
    mFrameSub = mGlobal->subscribe("/net_audio_frame", 20, &SpeakerNode::OnFrame,this);
	mSEM = make_shared<SoundEffectsMgr>(mGlobal);
}

SpeakerNode::~SpeakerNode()
{
	printf("speakernode exit.\r\n");
}

void SpeakerNode::ThreadInfo(char *function)
{
	pthread_detach(pthread_self());
	printf( "[info] %s, pid:%u, tid:%u, self:%lu,Time:%s %s\n",
		function, getpid(), syscall( SYS_gettid ), pthread_self(),
		__DATE__,__TIME__);
	prctl(PR_SET_NAME, function);
}


void SpeakerNode::OnCmd(const std_msgs::Int32::ConstPtr& msg)
{
        switch (msg->data){
		case SPEAKER_START:
			this->Start();
		   break;
        case SPEAKER_STOP:
			this->Stop();
            break;
        default:
            break;
        }
}

void SpeakerNode::OnFrame(const roller_eye::frameConstPtr& msg)
{
	if (msg->data.size()>AAC_BUF_LEN){
		return;
	}
	if (!IsRunning()){
		Start();
	}

	if(msg->data.size() <50){
        PLOG_INFO(SPEAKER_NODE_TAG,"data too short %d\n",msg->data.size());
	}

    lock_guard<mutex> lock(mRxMutex);

	if(MAX_FRAME_LEN <= mAACQue.size()){
		shared_ptr<AACData> AACFrame = mAACQue.front();
		mAACQue.pop();
	}
	shared_ptr<AACData> AACFrame(new AACData);
	if(nullptr != AACFrame){
		std::copy(msg->data.begin(), msg->data.end(), AACFrame->Data);
		AACFrame->len = msg->data.size();
		mAACQue.push(AACFrame);
	}
}

void SpeakerNode::Start()
{
	PLOG_INFO(SPEAKER_NODE_TAG,"call SpeakerNode::Start\n");
	lock_guard<mutex> lock(mCMDMutex);
    if (!IsRunning()){
   		mRunning  = true;

		auto DecoderThread = std::make_shared<std::thread>(&SpeakerNode::AAC2PCMLoop, this);
		DecoderThread->detach();
		auto PlayThread = std::make_shared<std::thread>(&SpeakerNode::PlayLoop, this);
		PlayThread->detach();

    }
	sleep(1);
	printf("SpeakerNode::Start ...\r\n");
}

void SpeakerNode::Stop()
{

	lock_guard<mutex> lock(mCMDMutex);
    mRunning = false;

	while(0 < mAACQue.size()){
		lock_guard<mutex> lock(mRxMutex);
		shared_ptr<AACData> AACFrame = mAACQue.front();
		mAACQue.pop();
	}
	while(0 < mPCMQue.size()){
		lock_guard<mutex> lock(mPcmMutex);
		shared_ptr<pcmData> PMCFrame = mPCMQue.front();
		mPCMQue.pop();
	}

	sleep(1);
	printf("SpeakerNode::Stop ...\r\n");
}

long timestamp()
{
	struct timeval tv;

	gettimeofday(&tv, NULL);
	return (tv.tv_sec*1000*1000 + tv.tv_usec);
}

void SpeakerNode::PlayLoop()
{
	lock_guard<mutex> lock(mPlayLoopMutex);
#if USE_SONIX_PLAY
    int ret = InitPlayTalk();
#else
	int ret = set_hardware_params(16000 , 1, 16);
#endif
	ThreadInfo((char *)"PlayLoop");
	if (ret<0){
		mRunning = false;
		return ;
	}
	static long lastTime = 0;
	uint8_t muteBuf[MUTE_BUF_LEN]={0};
    PLOG_INFO(SPEAKER_NODE_TAG, "call SpeakerNode::PlayLoop\n");
	int idles = 0;
	while(mRunning){
		while(mRunning && (0 == mPCMQue.size() ) ){
			usleep(5000);
			continue;
		}

		shared_ptr<pcmData> PCMFrame =GetPCM();
		if (nullptr != PCMFrame){
#if USE_SONIX_PLAY
			if (CanPlayTalk()){
				printf ("####%s,%d,\r\n",__FILE__,__LINE__);
				//2048
				for(int index=0;index < 20;index++ ){
					printf("%02X ",PCMFrame->pcmBuf[index]);
				}
				printf("\r\n");
				for(int index=2027;index < 2047;index++ ){
					printf("%02X ",PCMFrame->pcmBuf[index]);
				}
				printf("\r\n");

				ret = PlayTalkback((char*)(PCMFrame->pcmBuf), PCMFrame->len);
				printf ("####%s,%d,\r\n",__FILE__,__LINE__);
				if (ret < 0){
					printf ("@@@@@@@@@@@DeinitPlayTalk .!!!!!!!!!!!!!\r\n");
					DeinitPlayTalk();
				}

			}else{
				printf ("@@@@@@@@@@@InitPlayTalk .!!!!!!!!!!!!!\r\n");
				InitPlayTalk();
			}
#else
			if (!mHandle){
   			   PLOG_INFO(SPEAKER_NODE_TAG, "open alsa\n");
				set_hardware_params(16000 , 1, 16);
			}
			if (mHandle){
				PlayAudio(PCMFrame->pcmBuf, PCMFrame->len);
			}
		}
#endif
	}

#if USE_SONIX_PLAY
	printf ("@@@@@@@@@@@DeinitPlayTalk .!!!!!!!!!!!!!\r\n");
    DeinitPlayTalk ();
#else
	 if (nullptr != mHandle){
	 	snd_pcm_drain(mHandle);
	 	snd_pcm_close(mHandle);
	 	mHandle = nullptr;
	 }
#endif
	PLOG_INFO(SPEAKER_NODE_TAG, "[PlayLoop] thread exit\n");
}

pcmData* SpeakerNode::GetPcmBuf(std::queue<pcmData*>& que)
{
	lock_guard<mutex> lock(mPcmMutex);
	pcmData *pcmBuf = nullptr;
	if (que.size()>0){
		pcmBuf = que.front();
		que.pop();
	}

	return pcmBuf;
}

void SpeakerNode::PushPcm(pcmData* pcm, std::queue<pcmData*>& que)
{
	lock_guard<mutex> lock(mPcmMutex);
	que.push(pcm);
}

std::vector<uint8_t> SpeakerNode::GetFrame()
{
	std::vector<uint8_t> vFrame;
	lock_guard<mutex> lock(mRxMutex);

	if (!mframeQue.empty()){
		vFrame = mframeQue.front();
		mframeQue.pop();
	}
	return vFrame;
}

shared_ptr<AACData> SpeakerNode::GetAACFrame()
{
	shared_ptr<AACData> AACFrame = nullptr;
	lock_guard<mutex> lock(mRxMutex);

	if (!mAACQue.empty()){
		AACFrame = mAACQue.front();
		mAACQue.pop();
	}
	return AACFrame;
}

void SpeakerNode::PutPCM(shared_ptr<pcmData> PCMData)
{
	// PCM frame push
	lock_guard<mutex> lock(mPcmMutex);
	if(PCM_QUE_LEN <= mPCMQue.size()){
		shared_ptr<pcmData> PCMFrame = mPCMQue.front();
		mPCMQue.pop();
	}

	if(nullptr != PCMData){
		mPCMQue.push(PCMData);
	}
}

shared_ptr<pcmData> SpeakerNode::GetPCM()
{
	shared_ptr<pcmData> PCMFrame = nullptr;
	lock_guard<mutex> lock(mPcmMutex);

	if (!mPCMQue.empty()){
		PCMFrame = mPCMQue.front();
		mPCMQue.pop();
	}
	return PCMFrame;
}


void SpeakerNode::AAC2PCMLoop()
{
	lock_guard<mutex> lock(mAAC2PCMLoopMutex);

	ThreadInfo((char *)"AAC2PCMLoop");
	uint8_t aacBuf[AAC_BUF_LEN];

	while(mRunning)	{
		/*
		1, ACC QUE
		2, PCM QUE
		get ACC fram
		*/
		while(mRunning && (0 == mAACQue.size() ) ){
			usleep(5000);
			continue;
		}

		shared_ptr<AACData> AACFrame = GetAACFrame();

		if(nullptr != AACFrame){
			int len =0;
			//AAC --> PCM
			shared_ptr<pcmData> PCMDATA(new pcmData);
			if(nullptr != PCMDATA){
				memset(PCMDATA->pcmBuf,0,PCM_BUF_LEN);
				len = DecoderAAC::getInstance()->AAC2PCM(AACFrame->Data,AACFrame->len, PCMDATA->pcmBuf);
				if(100 < len){
					PCMDATA->len = len;
					PutPCM(PCMDATA);
				}
			}
		}
	}

	mRunning = false;

    PLOG_INFO(SPEAKER_NODE_TAG, "[AAC2PCMLoop] thread exit\n");
}

/*
 *   Underrun and suspend recovery
 */

static int xrun_recovery(snd_pcm_t *handle, int err)
{
    if (err == -EPIPE) {    /* under-run */
        err = snd_pcm_prepare(handle);
        if (err < 0)
            PLOG_INFO(SPEAKER_NODE_TAG, "Can't recovery from underrun, prepare failed: %s\n", snd_strerror(err));
        return 0;
    } else if (err == -ESTRPIPE) {
        while ((err = snd_pcm_resume(handle)) == -EAGAIN)
            usleep(10000);   /* wait until the suspend flag is released */
        if (err < 0) {
            err = snd_pcm_prepare(handle);
            if (err < 0)
            	PLOG_INFO(SPEAKER_NODE_TAG, "Can't recovery from suspend, prepare failed: %s\n", snd_strerror(err));
        }
        return 0;
    }
    return err;
}

int SpeakerNode::PlayAudio(uint8_t *buf, int len)
{
    int ret = 1;
	do{
		int error;
		if (!mHandle){
            PLOG_INFO(SPEAKER_NODE_TAG, "invalid handle %d\n",static_cast<int>(mFrames));
			break;
		}
		int eagainLoops = 0;
		while (len>0){
			ret = snd_pcm_writei(mHandle, buf, mFrames);
			if (ret == -EAGAIN){
				eagainLoops++;
				if (eagainLoops%200 == 0){
            		PLOG_INFO(SPEAKER_NODE_TAG, "eagainLoops %d\n",eagainLoops);
				}
				continue;
			}else{
				eagainLoops = 0;
			}

			if (ret<0){
				ret = snd_pcm_recover(mHandle, ret, 0);
				if (ret < 0) {
            		PLOG_INFO(SPEAKER_NODE_TAG, "Write error: %s\n", snd_strerror(ret));
					return -1;
				}
			}
			buf += mBufsize;
			len -= mBufsize;
		}
		ret = 0;
	}while(0);

    return ret;
}

int SpeakerNode::set_hardware_params(u32 sample_rate, int channels, int format_size)
{
	int rc;
	snd_pcm_uframes_t bufsize = 8192;
	snd_pcm_uframes_t exact_bufsize =bufsize;
	/* Open PCM device for playback */
	rc = snd_pcm_open(&mHandle, "default", SND_PCM_STREAM_PLAYBACK, 0);
	if (rc < 0) {
        PLOG_INFO(SPEAKER_NODE_TAG, "unable to open pcm device\n");
		return -1;
	}

	/* Allocate a hardware parameters object */
	snd_pcm_hw_params_alloca(&mParams);

	/* Fill it in with default values. */
	rc = snd_pcm_hw_params_any(mHandle, mParams);
	if (rc < 0) {
        PLOG_INFO(SPEAKER_NODE_TAG, "unable to Fill it in with default values.\n");
		goto err1;
	}


	/* Interleaved mode */
	rc = snd_pcm_hw_params_set_access(mHandle, mParams, SND_PCM_ACCESS_RW_INTERLEAVED);
	if (rc < 0) {
        PLOG_INFO(SPEAKER_NODE_TAG, "unable to Interleaved mode.\n");
		goto err1;
	}

	snd_pcm_format_t format;

    if (8 == format_size){
		format = SND_PCM_FORMAT_U8;
	}else if (16 == format_size)	{
		format = SND_PCM_FORMAT_S16_LE;
	}else if (24 == format_size)	{
		format = SND_PCM_FORMAT_U24_LE;
	}else if (32 == format_size)	{
		format = SND_PCM_FORMAT_U32_LE;
	}else{
        PLOG_INFO(SPEAKER_NODE_TAG, "SND_PCM_FORMAT_UNKNOWN.\n");
		format = SND_PCM_FORMAT_UNKNOWN;
		goto err1;
	}

	/* set format */
	rc = snd_pcm_hw_params_set_format(mHandle, mParams, format);
	if (rc < 0) {
        PLOG_INFO(SPEAKER_NODE_TAG, "unable to set format.\n");
		goto err1;
	}

	/* set channels (stero) */
	snd_pcm_hw_params_set_channels(mHandle, mParams, channels);
	if (rc < 0) {
        PLOG_INFO(SPEAKER_NODE_TAG, "unable to set channels (stero).\n");
		goto err1;
	}

	/* set sampling rate */
    int dir;
	rc = snd_pcm_hw_params_set_rate_near(mHandle, mParams, &sample_rate, &dir);
	if (rc < 0) {
        PLOG_INFO(SPEAKER_NODE_TAG,"unable to set sampling rate.\n");
		goto err1;
	}

	/* Write the parameters to the dirver */
	rc = snd_pcm_hw_params(mHandle, mParams);
	if (rc < 0) {
        PLOG_INFO(SPEAKER_NODE_TAG,"unable to set hw parameters: %s\n", snd_strerror(rc));
		goto err1;
	}

	snd_pcm_hw_params_get_period_size(mParams, &mFrames, &dir);
	mFrames = 1024;
	mBufsize = mFrames * (channels*format_size/8);

	if (snd_pcm_hw_params_set_buffer_size_near(mHandle, mParams, &exact_bufsize) < 0) {
        PLOG_INFO(SPEAKER_NODE_TAG,"Error setting buffersize.\n");
		goto err1;
	}

	if (exact_bufsize != bufsize){
        PLOG_INFO(SPEAKER_NODE_TAG, "exact_bufsize:%ld,bufsize:%ld\n", exact_bufsize, bufsize);
	}

	return 0;

err1:
	snd_pcm_close(mHandle);
	mHandle = nullptr;
	return -1;
}
} // namespace roller_eye
int main(int argc, char **argv)
{
  fprintf(stdout, "SpeakerNode run\n");
  ros::init(argc, argv, "SpeakerNode");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,  NAV_NODE_DEBUG_LEVEL);
  roller_eye::SpeakerNode node;
  ros::spin();
  return 0;
}

int main_func(int argc, char **argv)
{
	while(1){
		int status;
        pid_t pid = fork();
        if (pid == -1) {
            fprintf(stderr, "fork() error.errno:%d error:%sn", errno, strerror(errno));
            break;
        }
        if (pid == 0) {
           return main_func(argc, argv);
        }

        if (pid > 0) {
            pid = wait(&status);
            fprintf(stdout, "wait return\n");
        }

    }
	return 0;
}
