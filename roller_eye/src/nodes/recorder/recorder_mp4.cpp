
#include "roller_eye/recorder_mp4.h"


namespace roller_eye {

#define FILE_SAVE_TEST 0
#if FILE_SAVE_TEST
static FILE* fp_save264 = nullptr;
static FILE* fp_saveaac = nullptr;
#endif

RecorderMP4::RecorderMP4(const std::string filename, int width, int height, float fps)
{
    mDuration = 0;
    m_audioQueue = new boost::lockfree::spsc_queue<struct frame_t>(256);
    m_videoQueue = new boost::lockfree::spsc_queue<struct frame_t>(256);
    m_mainRunning = false;

    m_filename = filename;
    m_vWidth = width;
    m_vHeight = height;
    m_vFrateR = fps;

    #if FILE_SAVE_TEST
    fp_save264 = fopen("./testSaveMp4.h264", "wb");
    fp_saveaac = fopen("./testSaveMp4.aac", "wb");
    #endif
}

RecorderMP4::~RecorderMP4()
{
    delete m_audioQueue;
    delete m_videoQueue;
    closeMp4Encoder();

    #if FILE_SAVE_TEST
    fclose(fp_save264);
    fclose(fp_saveaac);
    #endif
}

void RecorderMP4::h264Callback(roller_eye::frameConstPtr frame)
{
    if (!m_mainRunning){
        return;
    }
    if (mDuration>0 && mAccDur>=(mDuration*m_vTimeScale)){
        return;
    }
    struct frame_t save;
    save.data = new std::vector<uint8_t>(frame->data);
    save.seq = frame->seq;
    save.stamp = frame->stamp;
    save.par3 = frame->par3;
    m_videoQueue->push(save);

    #if FILE_SAVE_TEST
    static int gotKeyframe = 0;
    struct timespec tpTime;
    uint64_t nowTime = 0;
    clock_gettime(CLOCK_MONOTONIC, &tpTime);
    nowTime = tpTime.tv_sec*1000 + tpTime.tv_nsec/1000000;

    if (frame->par3 == 1) {
      ROS_DEBUG("key");
      gotKeyframe = 1;
    }
    if (gotKeyframe == 1)
      fwrite(&frame->data[0], 1, frame->data.size(), fp_save264);
    else
      ROS_DEBUG("Waiting keyframe.");
    #endif
}

void RecorderMP4::aacCallback(roller_eye::frameConstPtr frame)
{
    if (!m_mainRunning){
        return;
    }
    if (mDuration>0 && mAccDur>=(mDuration*m_vTimeScale)){
        return;
    }
    struct frame_t save;
    save.data = new std::vector<uint8_t>(frame->data);
    save.seq = frame->seq;
    save.stamp = frame->stamp;
    save.par3 = frame->par3;
    m_audioQueue->push(save);

    #if FILE_SAVE_TEST
    struct timespec tpTime;
    uint64_t nowTime = 0;
    clock_gettime(CLOCK_MONOTONIC, &tpTime);
    nowTime = tpTime.tv_sec*1000 + tpTime.tv_nsec/1000000;

    fwrite(&frame->data[0], 1, frame->data.size(), fp_saveaac);
    #endif
}

bool RecorderMP4::isCompleted()
{
    return mAccDur>=(mDuration*m_vTimeScale);
}
void RecorderMP4::mainLoop()
{
    struct frame_t outframe;

    initMp4Encoder();
    while (m_mainRunning || !m_audioQueue->empty() || !m_videoQueue->empty()) {
        if (mDuration> 0 && mAccDur>=(mDuration*m_vTimeScale)){
            while(m_videoQueue->pop(outframe)){
                delete outframe.data;
            }
            while(m_audioQueue->pop(outframe)){
                delete outframe.data;
            }
            usleep(20*1000);
            continue;
        }
        if (!m_mainRunning) ROS_DEBUG("mp4 flushing..\n");

        if (!m_videoQueue->pop(outframe)) {
            usleep(20*1000);
        } else {
            if (1) {
              mp4VEncode(&outframe.data->front(), outframe.data->size(), outframe.stamp, outframe.par3);
            } else {
              ROS_DEBUG("---audio not ready.\n");
            }
            delete outframe.data;
        }

        if (!m_audioQueue->pop(outframe)) {
            usleep(20*1000);
        } else {
            m_audioReady = true;
            mp4AEncode(&outframe.data->front(), outframe.data->size(), outframe.stamp);

            delete outframe.data;
        }

    }
    closeMp4Encoder();
}

void RecorderMP4::startLoop()
{
    if (m_mainRunning == true) {
        return;
    }
    mAccDur = 0;
    m_mainRunning = true;
    m_mainThread = std::thread(&RecorderMP4::mainLoop, this);
}

void RecorderMP4::stopLoop()
{
    if(!m_mainRunning) {
        return;
    }

    m_mainRunning = false;
    m_mainThread.join();
}

static int getSPSLen(const uint8_t *naluData)
{
    int len = 0;
    while (true) {
        if (START_PPS(naluData)) {
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
            break;
        }
        else if (START_IDR(naluData)) {
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
            break;
        }
        naluData++;
        len++;
    }
    return len;
}

int RecorderMP4::getFileDuration(const char *fileName)
{
    MP4FileHandle hd = MP4Read(fileName);
    if (hd == MP4_INVALID_FILE_HANDLE) {
        ROS_DEBUG("MP4 Read failed.\n");
        return -1;
    }
    MP4Duration dur = MP4GetDuration(hd);
    uint32_t ts = MP4GetTimeScale(hd);
    MP4Close(hd);

    return dur/ts;
}

int RecorderMP4::initMp4Encoder()
{
    mAccDur = 0;
    m_vTimeScale = 90000;
    m_vFrameDur = 300;
    m_vTrackId = 0;
    m_aTrackId = 0;

    m_mp4FHandle = MP4Create(m_filename.c_str(), 0);
    if (m_mp4FHandle == MP4_INVALID_FILE_HANDLE) {
        ROS_DEBUG("error : MP4Create  \n");
        return -1;
    }
    MP4SetTimeScale(m_mp4FHandle, m_vTimeScale);

    //-- audio track-----------------------------
    m_aTrackId = MP4AddAudioTrack(m_mp4FHandle, RECORD_SAMPLE_RATE, AUDIO_DURATION, MP4_MPEG4_AUDIO_TYPE);
    if (m_aTrackId == MP4_INVALID_TRACK_ID){
        ROS_DEBUG("error : MP4AddAudioTrack  \n");
        return -1;
    }
    MP4SetAudioProfileLevel(m_mp4FHandle, 0x2);

    uint8_t aacConfig[2] = {0x13, 0x88};
    MP4SetTrackESConfiguration(m_mp4FHandle, m_aTrackId, aacConfig,2);

    m_audioReady = false;
    m_encodedIFrame = false;
    m_audioStamp=0;
	m_videoStamp=0;
	s_nAudioJitter = 0;

    ROS_DEBUG("ok : initMp4Encoder filename: %s\n\n", m_filename.c_str());
    return 0;
}

int RecorderMP4::mp4VEncode(uint8_t * naluData ,int naluSize, uint64_t timestamp, int32_t isKeyframe)
{
    MP4Duration nDuration = 0;
    int index = -1;

    if(START_SPS(naluData)){
        index = _NALU_SPS_;
    }
    if(index !=_NALU_SPS_ && m_vTrackId == MP4_INVALID_TRACK_ID){
        return index;
    }

    if(START_P(naluData)){
        index = _NALU_P_;
    }
    else if(START_PPS(naluData)){
        index = _NALU_PPS_;
    }
    else if(START_IDR(naluData)){
        index = _NALU_I_;
    }

    static int nCounter =0;
    switch(index){
        case _NALU_SPS_:
            if(m_vTrackId == MP4_INVALID_TRACK_ID) {
                m_vTrackId = MP4AddH264VideoTrack(
                    m_mp4FHandle,
                    m_vTimeScale,
                    static_cast<MP4Duration>(m_vTimeScale / m_vFrateR),
                    m_vWidth,     // width
                    m_vHeight,    // height
                    naluData[5], // sps[1] AVCProfileIndication
                    naluData[6], // sps[2] profile_compat
                    naluData[7], // sps[3] AVCLevelIndication
                    3);           // 4 bytes length before each NAL unit
                if (m_vTrackId == MP4_INVALID_TRACK_ID)  {
                    return -1;
                }
                nCounter ++;
                MP4SetVideoProfileLevel(m_mp4FHandle, 0x7F); //  Simple Profile @ Level 3
            }
            if (isKeyframe) {
                int spslen = getSPSLen(naluData);
                MP4AddH264SequenceParameterSet(m_mp4FHandle,m_vTrackId, naluData +4, spslen-4);
                naluData += spslen;

                int ppslen = getPPSLen(naluData);
                MP4AddH264PictureParameterSet(m_mp4FHandle,m_vTrackId, naluData +4, ppslen-4);
                naluData += ppslen;

                if (START_SEI(naluData)){
                    naluData += getSEILen(naluData);
                }

                if (START_IDR(naluData)){
                    goto L1;
                }
            }

            break;
        case _NALU_PPS_:
            ROS_DEBUG("get _NALU_PPS_\n");
            MP4AddH264PictureParameterSet(m_mp4FHandle,m_vTrackId,naluData+4,naluSize-4);
            break;
        case _NALU_I_:
        {
L1:
        #ifdef APP_ARCH_X86
            uint8_t * IFrameData = (uint8_t *)malloc(naluSize+1);
            IFrameData[0] = (naluSize-3) >>24;
            IFrameData[1] = (naluSize-3) >>16;
            IFrameData[2] = (naluSize-3) >>8;
            IFrameData[3] = (naluSize-3) &0xff;

            memcpy(IFrameData+4,naluData+3,naluSize-3);

            nDuration = ((m_videoStamp==0||timestamp==0)?m_vTimeScale/m_vFrateR:(timestamp-m_videoStamp)*(m_vTimeScale/1000));
            m_videoStamp = timestamp;
            if(!MP4WriteSample(m_mp4FHandle, m_vTrackId, IFrameData, naluSize+1, nDuration, 0, 1)){
                return -1;
            }
            free(IFrameData);
        #else
            naluData[0] = ((naluSize-4) >>24) & 0xFF;
            naluData[1] = ((naluSize-4) >>16) & 0xFF;
            naluData[2] = ((naluSize-4) >>8) & 0xFF;
            naluData[3] = (naluSize-4) & 0xff;

            nDuration = ((m_videoStamp==0||timestamp==0)?m_vTimeScale/m_vFrateR:(timestamp-m_videoStamp)*(m_vTimeScale/1000));
            m_videoStamp = timestamp;
            if(!MP4WriteSample(m_mp4FHandle, m_vTrackId, naluData, naluSize, nDuration, 0, 1)){
                return -1;
            }

                nCounter ++;
        #endif
            m_encodedIFrame = true;
            break;
        }
        case _NALU_P_:
        {
            if (m_encodedIFrame == false) {
                ROS_DEBUG("Not encode i frame yet, timestamp: %ld\n", timestamp);
                return -1;
            }
            naluData[0] = ((naluSize-4) >>24) & 0xFF;
            naluData[1] = ((naluSize-4) >>16) & 0xFF;
            naluData[2] = ((naluSize-4) >>8) & 0xFF;
            naluData[3] = (naluSize-4) &0xff;

            nDuration = ((m_videoStamp==0||timestamp==0)?m_vTimeScale/m_vFrateR:(timestamp-m_videoStamp)*(m_vTimeScale/1000));
            m_videoStamp = timestamp;
            if(!MP4WriteSample(m_mp4FHandle, m_vTrackId, naluData, naluSize, nDuration, 0, 0)){
                return -1;
            }

            nCounter++;
            break;
        }
        default:
            ROS_DEBUG("mp4VEncode not defined, timestamp: %ld\n", timestamp);
    }
    mAccDur += nDuration;
    return 0;
}

int RecorderMP4::mp4AEncode(uint8_t * data ,int len, uint64_t timestamp)
{
    if (m_encodedIFrame == false) {
        return -1;
    }

    MP4Duration nDuration = ((m_audioStamp==0||timestamp==0)?AUDIO_DURATION:(timestamp-m_audioStamp)*RECORD_SAMPLE_RATE/1000);
	s_nAudioJitter+=((int64_t)nDuration-(int64_t)AUDIO_DURATION)*1000/RECORD_SAMPLE_RATE;

    nDuration = AUDIO_DURATION;
    if (s_nAudioJitter>MAX_AUDIO_JITTER) {
        nDuration+=s_nAudioJitter*RECORD_SAMPLE_RATE/1000;
        s_nAudioJitter=0;
        ROS_DEBUG("---max jitter,duration=%lu\n", nDuration);
    }

    MP4WriteSample(m_mp4FHandle, m_aTrackId, data + 7, len - 7, nDuration, 0, 1);
    m_vFrameDur += nDuration;
	m_audioStamp = timestamp;

    return 0;
}

void RecorderMP4::closeMp4Encoder()
{
    if (m_mp4FHandle != MP4_INVALID_FILE_HANDLE) {
        MP4Close(m_mp4FHandle,0);
        MP4Optimize(m_filename.c_str());   //mv moov add by ltl 2021-04-16
        m_mp4FHandle = MP4_INVALID_FILE_HANDLE;
    }

    m_audioReady = false;
    m_encodedIFrame = false;
    m_audioStamp=0;
    m_videoStamp=0;
    s_nAudioJitter = 0;
}


} // namespace roller_eye
