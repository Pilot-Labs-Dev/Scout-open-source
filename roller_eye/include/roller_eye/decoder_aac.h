#pragma once

extern "C" {
    #include "libavformat/avformat.h"
    #include "libavcodec/avcodec.h"
    #include "libswresample/swresample.h"
    #include "libavutil/avutil.h"
    #include "libswscale/swscale.h"
}
//#include"roller_eye/single_class.h"

namespace roller_eye
{
    class DecoderAAC/*:public SingleClass<DecoderAAC>*/{
                //friend class SingleClass<DecoderAAC>;
    private:
        /* data */
        DecoderAAC();
        ~DecoderAAC();
    public:
        static  DecoderAAC* getInstance();
        bool initAACDecoderWithSampleRate(int sampleRate, int channel, int bit);
        void releaseAACDecoder();
        int  AAC2PCM(uint8_t * data, int len,  uint8_t * outBuf);
        
    private:
        AVFrame *mAacFrame = nullptr;
        AVCodec *mAacCodec = nullptr;
        AVCodecContext *mAacCodecCtx = nullptr;
        AVPacket mAacPacket;
        int mSampleRate;     
    };    

}