#ifndef __ROLLER_EYE_RTMP_PUB_H__
#define __ROLLER_EYE_RTMP_PUB_H__

#include <ros/ros.h>
#include "roller_eye/frame.h"
#include <thread>
#include<memory>
#include<functional>
#include<atomic>
#include"roller_eye/block_queue.h"
#include "roller_eye/srs_librtmp.h"
#include "roller_eye/upload_cache_mgr.h"
#include <dynamic_reconfigure/client.h>
#include <roller_eye/ImageConfig.h>

namespace roller_eye {


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

enum{
    RTMP_FRAME_VIDEO_TYPE,
    RTMP_FRAME_AUDIO_TYPE,
    RTMP_FRAME_DUMMY_TYPE
};

#define RTMP_FLAG_AUDIO_NONE 0x01

struct frame_t {
    std::vector<uint8_t> data;
    uint32_t seq;
    uint32_t oseq;
    uint64_t stamp;
    int32_t par1;
    int32_t par2;
    int32_t par3;
    int type;
};

// const char *rtmp_url = "rtmp://127.0.0.1/live/livestream";
// const char *rtmp_url = "rtmp://publisherName:123456@118.24.178.237:1935/live/testRtmp";
// const char *rtmp_url = "rtmp://118.24.178.237:1935/live/testRtmp pubUser=publisherName pubPasswd=123456";
// const char *rtmp_url = "rtmp://118.24.178.237:1935/live?doPublish=12345/testRtmpAA";
// const char *rtmp_url = "rtmp://118.24.178.237:1935/live/testRtmp1218";
//const std::string WOWZA_APP_URL = "rtmp://118.24.178.237:1935/live/";
const std::string WOWZA_APP_URL = "rtmp://localhost:60003/";
const std::string WOWZA_USER_NAME = "publisherName";
const std::string WOWZA_PASSWORD = "12345";

typedef std::function<void(std::string& ,std::string&,bool)> RtmpStopCb;
class RtmpPub
{
public:
    RtmpPub(const std::string app,const std::string stream_name,bool cache=false,bool live=false,int duration=-1,uint32_t flags=0);
    ~RtmpPub();
    void h264Callback(frameConstPtr frame);
    void aacCallback(frameConstPtr frame);
    void startLoop();
    void stopLoop();
    bool isLive();
    bool hasCache();
    bool compareApp(const std::string &app,const std::string& name);
    void setEndCallback(const RtmpStopCb &cb);
    bool isNetworkOK();
private:
    void mainLoop();
    void loadCacheLoop();
    int h264WriteFrame(uint8_t* data, int size, uint32_t dts, uint32_t pts);
    int audioWriteFrame(frame_t& frame);
    void videoWriteFrame(frame_t& frame, bool& sent_spspps);
    int initRtmpClient(const std::string url, const std::string& username, const std::string& passwd);

    bool mLive;
	std::atomic<bool> m_mainRunning;
    std::thread m_mainThread;
    std::thread m_cacheThread;
    shared_ptr<CacheFile> mCacheReader;

    BlockQueue<struct frame_t> *m_frameQueue;

    srs_rtmp_t m_rtmp;
    bool mOnlyKeyMode;
    uint32_t mTimestampDelay;
    uint32_t mPreTimestamp;
    int mAudioFrameCounter;
    shared_ptr<CacheFile> mCache;
    uint32_t mSeq;
    bool mIOError;

    std::string mApp;
    std::string mStreamName;
    int mDuration;
    uint32_t mFlags;

    RtmpStopCb mStopCB;
    dynamic_reconfigure::Client<roller_eye::ImageConfig> mCfgClient;
};

} // namespace roller_eye

#endif //__ROLLER_EYE_RTMP_PUB_H__