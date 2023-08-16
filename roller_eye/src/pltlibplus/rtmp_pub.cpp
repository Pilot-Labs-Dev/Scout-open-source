
#include "roller_eye/rtmp_pub.h"
#include"roller_eye/plt_assert.h"
#include"roller_eye/plt_tools.h"
#include "roller_eye/system_define.h"
namespace roller_eye {

#define FAST_NX             5
#define RTMP_TAG        "RTMP"

RtmpPub::RtmpPub(const std::string app,const std::string stream_name,bool cache,bool live,int duration,uint32_t flags):
mLive(live),mSeq(0),mIOError(false),mApp(app),mStreamName(stream_name),mDuration(duration),mFlags(flags),
mStopCB(nullptr), mCfgClient(PARAM_VIDEO_PATH)
{
    m_frameQueue = new BlockQueue<struct frame_t>(128);
    m_mainRunning = false;
    mTimestampDelay=0;
    mPreTimestamp=0;
    plt_assert(!stream_name.empty()&&!app.empty());
    initRtmpClient(WOWZA_APP_URL + app + "/"+stream_name, WOWZA_USER_NAME, WOWZA_PASSWORD);
    if(live&&cache){
        mCache=CacheMgr::getInstance()->newWriteCache(stream_name);
        if(!mCache){
            PLOG_ERROR(RTMP_TAG,"Create write cache fail\n");
        }
    }
}

RtmpPub::~RtmpPub()
{
    if(m_mainRunning){
        stopLoop();
    }
    if(!mLive){
         m_cacheThread.join();
     }
    m_mainThread.join();
    delete m_frameQueue;
    srs_rtmp_destroy(m_rtmp);
    PLOG_DEBUG("RTMP_TAG","RTMP [%s/%s] close",mApp.c_str(),mStreamName.c_str());
}

void RtmpPub::h264Callback(roller_eye::frameConstPtr frame)
{
    if(!mLive||!m_mainRunning){
        return;
    }
    struct frame_t save;
    save.data = frame->data;
    save.seq = frame->seq;
    save.oseq = frame->oseq;
    save.stamp = frame->stamp;
    save.par3 = frame->par3;
    save.type=RTMP_FRAME_VIDEO_TYPE;
    m_frameQueue->pushNoblock(save);
    if(mCache){
        mCache->write(&save.data[0],save.data.size(),save.par3?CACHE_VIDEO_I_BUFF:CACHE_VIDEO_P_BUFF,save.stamp);
    }
}

void RtmpPub::aacCallback(roller_eye::frameConstPtr frame)
{
    if(!mLive||!m_mainRunning){
        return;
    }
    struct frame_t save;
    save.data = frame->data;
    save.seq = frame->seq;
    save.stamp = frame->stamp;
    save.par3 = frame->par3;
    save.type=RTMP_FRAME_AUDIO_TYPE;
    m_frameQueue->pushNoblock(save);
     if(mCache){
        mCache->write(&save.data[0],save.data.size(),CACHE_AUDIO_BUFF,save.stamp);
    }
}

void RtmpPub::loadCacheLoop()
{
    if(m_rtmp==nullptr){
        stopLoop();
        return;
    }

    int ret;
     struct frame_t save;
     Cache cache;
     mCacheReader=CacheMgr::getInstance()->newReadCache(mStreamName);

     if(!mCacheReader){
         PLOG_ERROR(RTMP_TAG,"Create read cache fail\n");
         mIOError=true;
         stopLoop();
         return;
     }

    uint64_t preStamp=0;
    uint64_t preMono=0;
    bool firstFrame=true;
    int64_t timeoffset;

    PLOG_INFO(RTMP_TAG,"start load cache file [%s] \n",mStreamName.c_str());
    while(m_mainRunning){
        if((ret=mCacheReader->read(cache))!=CACHE_OK){
            break;
        }
        save.data = cache.buff;
        save.seq = mSeq++;
        save.stamp = cache.stamp;

        if(cache.type==CACHE_VIDEO_I_BUFF||cache.type==CACHE_VIDEO_P_BUFF){
            save.par3=(cache.type==CACHE_VIDEO_I_BUFF?1:0);
            save.type=RTMP_FRAME_VIDEO_TYPE;
        }else if(cache.type==CACHE_AUDIO_BUFF){
            save.type=RTMP_FRAME_AUDIO_TYPE;
        }else{
            plt_assert(false);
        }

        if(firstFrame){
            firstFrame=false;
        }else{
            timeoffset=(int64_t)cache.stamp-(int64_t)preStamp-((int64_t)plt_get_mono_time_ms()-(int64_t)preMono);
            if(timeoffset>0){
                //PLOG_DEBUG(RTMP_TAG,"sleep %d ms",(int)timeoffset);
                usleep(timeoffset*1000);
            }
        }
        preStamp=cache.stamp;
        preMono=plt_get_mono_time_ms();
        m_frameQueue->push(save);
    }
    if(ret<0){
        PLOG_ERROR(RTMP_TAG,"Read cache fail,ret=%d\n",ret);
        if(ret!=-1){//if ret==-1,the cache file is bad,else io error
            mIOError=true;
        }
    }
     PLOG_DEBUG(RTMP_TAG,"load cache file [%s] done \n",mStreamName.c_str());
    stopLoop();
}

static int getSPSLen(const uint8_t *naluData)
{
    int len = 0;
    while (true) {
        if (START_PPS(naluData)) {
            // printf("Got pps. ret sps len: %d\n", len);
            break;
        }
        naluData++;
        len++;
    }
    return len;
}
static int getPPSLen(const uint8_t *naluData)
{
    int len = 0;
    while (true) {
        if (START_SEI(naluData)) {
            // printf("Got sei. ret pps len: %d\n", len);
            break;
        }
        else if (START_IDR(naluData)) {
            // printf("Got idr. ret pps len: %d\n", len);
            break;
        }
        naluData++;
        len++;
    }
    return len;
}
static int getSEILen(const uint8_t *naluData)
{
    int len = 0;
    while (true) {
        if (START_IDR(naluData)) {
            // printf("Got idr. ret sei len: %d\n", len);
            break;
        }
        naluData++;
        len++;
    }
    return len;
}

void RtmpPub::mainLoop()
{
    PLOG_DEBUG(RTMP_TAG,"RtmpPub::mainLoop" );
    struct frame_t outframe;
    //bool audioReady = false;
    bool sent_spspps = false;
    int64_t lastStamp=-1;
    int frames = 0;
    int fps = 30;
    roller_eye::ImageConfig cfg;
    int totalFrames = fps*mDuration/1000;
    if (mCfgClient.getCurrentConfiguration(cfg, ros::Duration(5))) {
        fps = cfg.image_fps_den/cfg.image_fps_num;
    }
    totalFrames = fps*mDuration/1000;
    mOnlyKeyMode = false;

    PLOG_INFO(RTMP_TAG,"RTMP fps:%d totalFrames:%d\n",fps, totalFrames);

    uint32_t oseq = 0;
    while (m_mainRunning || !m_frameQueue->empty()) {
        m_frameQueue->pop(outframe);
        if(outframe.type==RTMP_FRAME_DUMMY_TYPE){
            continue;
        }
        if(outframe.type==RTMP_FRAME_VIDEO_TYPE){
            if (mDuration>0 &&  0==oseq){
                oseq = outframe.oseq;
                lastStamp = outframe.stamp;
            }

            if(mDuration>0&&lastStamp>=0){
                int dur=(int64_t)outframe.stamp-lastStamp;
                if(dur>0){
                    if((mDuration-=dur)<=0){
                        m_mainRunning=false;
                        PLOG_INFO(RTMP_TAG,"RTMP [%s] timeout\n",mStreamName.c_str());
                    }
                }
            }
            lastStamp=(int64_t)outframe.stamp;
        }
        if ((!mIOError||(mLive&&!mCache))/*&&audioReady*/) {//if write to cache and occur io error,stop send data to server,because later will resend cache data to server.
            if(outframe.type==RTMP_FRAME_VIDEO_TYPE){
                videoWriteFrame(outframe, sent_spspps);
                //frames++;
            }else{
                PLOG_INFO(RTMP_TAG,"RTMP [%s] audio call\n",mStreamName.c_str());
                if ( (!mOnlyKeyMode) || (mOnlyKeyMode&& (mAudioFrameCounter++%FAST_NX==0))){
                    audioWriteFrame(outframe);
                }
            }
        }
    }

    if (m_rtmp != nullptr && srs_rtmp_unpublish_stream(m_rtmp) != 0) {
        PLOG_ERROR(RTMP_TAG,"unpublish stream failed.");
    }
    PLOG_DEBUG(RTMP_TAG,"RtmpPub::mainLoop exit" );

     if(mStopCB){
        auto th=std::thread([](RtmpStopCb cb, std::string app,std::string name,bool ok){
            cb(app,name,ok);
        },mStopCB,mApp,mStreamName,mIOError);
        th.detach();
    }
}

void RtmpPub::startLoop()
{
    if (m_rtmp == nullptr) {
        mIOError=true;
    }
    if (m_mainRunning == true) {
        return;
    }
    m_mainRunning = true;
    m_mainThread = std::thread(&RtmpPub::mainLoop, this);
    if(!mLive){
        m_cacheThread=std::thread(&RtmpPub::loadCacheLoop,this);
    }
}

void RtmpPub::stopLoop()
{
    if(!m_mainRunning) {
        return;
    }

    m_mainRunning = false;
    struct frame_t outframe;
    outframe.type=RTMP_FRAME_DUMMY_TYPE;
    m_frameQueue->pushNoblock(outframe);
}

bool RtmpPub::isLive()
{
    return mLive;
}
bool RtmpPub::isNetworkOK()
{
    return m_rtmp!=nullptr;
}
bool RtmpPub::hasCache()
{
    return mCache!=nullptr;
}
bool RtmpPub::compareApp(const std::string &app,const std::string& name)
{
    return (app==mApp&&name==mStreamName);
}
void RtmpPub::setEndCallback(const RtmpStopCb &cb)
{
    mStopCB=cb;
}
void RtmpPub::videoWriteFrame(frame_t& frame, bool& sent_spspps)
{
    uint8_t *data = &frame.data.front();
    int size = frame.data.size();
    if (0 == frame.seq){
        PLOG_INFO(RTMP_TAG,"video first picture!");
    }

    if (frame.par3) {
        mPreTimestamp=frame.stamp;
        int spslen = getSPSLen(data);
        if (!sent_spspps)
            h264WriteFrame(data, spslen, frame.stamp, frame.stamp);
        data += spslen;

        int ppslen = getPPSLen(data);
        if (!sent_spspps)
            h264WriteFrame(data, ppslen, frame.stamp, frame.stamp);
        data += ppslen;

        if (!sent_spspps)
            sent_spspps = true;

        int seilen = 0;
        if (START_SEI(data)){
            seilen = getSEILen(data);
            data += seilen;
        }
        h264WriteFrame(data, size - spslen - ppslen - seilen, frame.stamp, frame.stamp);
    } else {
        if (mOnlyKeyMode) {
            mTimestampDelay+=(frame.stamp-mPreTimestamp)*(FAST_NX-1)/FAST_NX;
            mPreTimestamp=frame.stamp;
            return;
        }
        if (sent_spspps)
            h264WriteFrame(data, size, frame.stamp, frame.stamp);
    }
}

int RtmpPub::h264WriteFrame(uint8_t* data, int size, uint32_t dts, uint32_t pts)
{
    // send out the h264 packet over RTMP
    if(m_rtmp == nullptr){
        return -1;
    }
    int ret = srs_h264_write_raw_frames(m_rtmp, (char*)data, size, dts-mTimestampDelay, pts-mTimestampDelay);
    if (ret != 0) {
        if (srs_h264_is_dvbsp_error(ret)) {
            PLOG_DEBUG(RTMP_TAG,"ignore drop video error, code=%d", ret);
        } else if (srs_h264_is_duplicated_sps_error(ret)) {
            PLOG_DEBUG(RTMP_TAG,"ignore duplicated sps, code=%d", ret);
        } else if (srs_h264_is_duplicated_pps_error(ret)) {
            PLOG_DEBUG(RTMP_TAG,"ignore duplicated pps, code=%d", ret);
        } else {
            mIOError=true;
            PLOG_DEBUG(RTMP_TAG,"send h264 raw data failed. ret=%d", ret);
            return -1;
        }
    }

    return 0;
}

int RtmpPub::audioWriteFrame(frame_t& frame)
{
    if(mFlags&RTMP_FLAG_AUDIO_NONE){
        return 0;
    }
    uint8_t *data = &frame.data.front();
    int size = frame.data.size();
    uint32_t timestamp = frame.stamp;
    // 10 = AAC
    char sound_format = 10;
    // 0 = 5.5 kHz
    // 1 = 11 kHz
    // 2 = 22 kHz
    // 3 = 44 kHz
    char sound_rate = 2;
    // 1 = 16-bit samples
    char sound_size = 1;
    // 1 = Stereo sound
    char sound_type = 0;

    PLOG_INFO(RTMP_TAG,"audioWriteFrame %x\n",m_rtmp);
    int ret = 0;
    if (m_rtmp!=nullptr && (ret = srs_audio_write_raw_frame(m_rtmp,
        sound_format, sound_rate, sound_size, sound_type,
        (char*)data, size, timestamp-mTimestampDelay)) != 0
    ) {
        mIOError=true;
        PLOG_DEBUG(RTMP_TAG,"send audio raw data failed. ret=%d", ret);
        return -1;
    }

    // srs_human_trace("sent packet: type=%s, time=%d, size=%d, codec=%d, rate=%d, sample=%d, channel=%d",
    //     srs_human_flv_tag_type2string(SRS_RTMP_TYPE_AUDIO), timestamp, size, sound_format, sound_rate, sound_size,
    //     sound_type);

    return 0;
}

int RtmpPub::initRtmpClient(const std::string url, const std::string& username, const std::string& passwd)
{
    // connect rtmp context
    m_rtmp = srs_rtmp_create(url.c_str());

    if (srs_rtmp_handshake(m_rtmp) != 0) {
        PLOG_ERROR(RTMP_TAG,"simple handshake failed.");
        goto rtmp_destroy;
    }

    // set after handshake success
    srs_rtmp_set_user_passwd(m_rtmp, username.c_str(), passwd.c_str());

    if (srs_rtmp_connect_app(m_rtmp) != 0) {
        PLOG_ERROR(RTMP_TAG,"connect vhost/app failed.");
        goto rtmp_destroy;
    }

    if (srs_rtmp_publish_stream(m_rtmp) != 0) {
        PLOG_ERROR(RTMP_TAG,"publish stream failed.");
        goto rtmp_destroy;
    }

    PLOG_DEBUG(RTMP_TAG,"publish stream success");
    return 0;

rtmp_destroy:
    srs_rtmp_destroy(m_rtmp);
    m_rtmp = nullptr;
    return -1;
}


} // namespace roller_eye
