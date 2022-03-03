#ifndef __ROLLER_EYE_RECORDER_MP4_H__
#define __ROLLER_EYE_RECORDER_MP4_H__

#include <ros/ros.h>
#include "roller_eye/frame.h"
#include <thread>
#include <boost/lockfree/spsc_queue.hpp>
#include <mp4v2/mp4v2.h>

namespace roller_eye {

#define  _NALU_SPS_  0
#define  _NALU_PPS_  1
#define  _NALU_I_    2
#define  _NALU_P_    3

/*#ifdef APP_ARCH_X86
#define SPS_LEN  27
#define PPS_LEN  8
#define SEI_LEN  631
#else //rk_mpp
#define SPS_LEN  27
#define PPS_LEN  8
#define SEI_LEN  266
#endif */

#define START_SPS(data) \
(data[0]==0 && data[1]==0 && data[2]==0 && data[3]==1 && (data[4]==0x67 || data[4]==0x27))

#define START_PPS(data) \
(data[0]==0 && data[1]==0 && data[2]==0 && data[3]==1 && (data[4]==0x68 || data[4]==0x28))

#define START_P(data) \
(data[0]==0 && data[1]==0 && data[2]==0 && data[3]==1 && (data[4]==0x41 || data[4]==0x21))

#define START_SEI(data) \
((data[0]==0 && data[1]==0 && data[2]==1 && data[3]==0x6) \
|| (data[0]==0 && data[1]==0 && data[2]==0 && data[3]==1 && data[4]==0x6))

#define START_IDR(data) \
((data[0]==0 && data[1]==0 && data[2]==1 && data[3]==0x65) \
|| (data[0]==0 && data[1]==0 && data[2]==0 && data[3]==1 && data[4]==0x25))

#define RECORD_SAMPLE_RATE  22050 //16000
#define AUDIO_DURATION      1024
#define MAX_AUDIO_JITTER    100


struct frame_t {
    std::vector<uint8_t> *data;
    uint32_t seq;
    uint64_t stamp;
    int32_t par1;
    int32_t par2;
    int32_t par3;
};

class RecorderMP4
{
public:
    RecorderMP4(const std::string filename, int w, int h, float fps=30);
    ~RecorderMP4();
    void h264Callback(frameConstPtr frame);
    void aacCallback(frameConstPtr frame);
    void startLoop();
    void stopLoop();
    void setDuration(int duration){mDuration = duration;}
    bool isRunning(){return m_mainRunning;}
    bool isCompleted();

    static int getFileDuration(const char* filename);

private:
    void mainLoop();
    int initMp4Encoder();
    int mp4VEncode(uint8_t *data, int len, uint64_t timestamp, int32_t isKeyframe);
    int mp4AEncode(uint8_t *data, int len, uint64_t timestamp);
    void closeMp4Encoder();

    std::string m_filename;
	std::atomic<bool> m_mainRunning;
    std::thread m_mainThread;

    boost::lockfree::spsc_queue<struct frame_t> *m_audioQueue;
    boost::lockfree::spsc_queue<struct frame_t> *m_videoQueue;
    // boost::lockfree::spsc_queue<struct frame_t, boost::lockfree::capacity<1024> > *spsc_queue;

    MP4FileHandle m_mp4FHandle;
    MP4TrackId m_vTrackId, m_aTrackId;
    int m_vWidth, m_vHeight;
    float m_vFrateR, m_vTimeScale;
    double m_vFrameDur;
    int mAccDur;
    int mDuration;
    bool m_audioReady;
    bool m_encodedIFrame;
    MP4Duration m_videoStamp;
    MP4Duration m_audioStamp;
    int64_t s_nAudioJitter;
};

} // namespace roller_eye

#endif //__ROLLER_EYE_RECORDER_MP4_H__