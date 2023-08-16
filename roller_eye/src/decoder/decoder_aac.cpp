#include "roller_eye/decoder_aac.h"
namespace roller_eye
{
    DecoderAAC::DecoderAAC()
    {
        initAACDecoderWithSampleRate(16000, 1, 16);
    }

    DecoderAAC::~DecoderAAC()
    {
        releaseAACDecoder();
    }

    DecoderAAC* DecoderAAC::getInstance()
    {
        static DecoderAAC* inst=nullptr;
        if(inst==nullptr){
            inst=new DecoderAAC();
        }
        return inst;
    }

bool DecoderAAC::initAACDecoderWithSampleRate(int sampleRate, int channel, int bit)
 {
    av_register_all();
    avformat_network_init();
    mAacCodec = avcodec_find_decoder(AV_CODEC_ID_AAC);
    av_init_packet(&mAacPacket);

    if (mAacCodec != NULL) {
        mAacCodecCtx = avcodec_alloc_context3(mAacCodec);

        mAacCodecCtx->codec_type = AVMEDIA_TYPE_AUDIO;
        mAacCodecCtx->sample_rate = sampleRate;
        mAacCodecCtx->channels = channel;
        //mAacCodecCtx->bit_rate = 256000;
        mAacCodecCtx->channel_layout = AV_CH_LAYOUT_MONO;

       mSampleRate = sampleRate;
       channel = channel;
       bit = bit;

        if (avcodec_open2(mAacCodecCtx, mAacCodec, NULL) >= 0) {
            //mAacFrame = av_frame_alloc();
        }
    }
    return (bool)mAacFrame;
}


int DecoderAAC::AAC2PCM(uint8_t * data, int len, uint8_t * outBuf)
 {
    mAacPacket.data = data;
    mAacPacket.size =  len;
    char errorbuf[1024] = {0};
    int ret = -1;
    if (&mAacPacket) {
        int result = avcodec_send_packet(mAacCodecCtx, &mAacPacket);
        if (result != 0){
            av_strerror(result, errorbuf, sizeof(errorbuf));
            return -1;
        }
        AVFrame *mAacFrame = nullptr;
        mAacFrame = av_frame_alloc();
        result = avcodec_receive_frame(mAacCodecCtx, mAacFrame);
        if ( 0 == result ) {
#if 1
            struct SwrContext * au_convert_ctx = swr_alloc_set_opts(nullptr,
                                                AV_CH_LAYOUT_MONO,
                                                AV_SAMPLE_FMT_S16,
                                                mSampleRate,
                                                mAacCodecCtx->channel_layout,
                                                mAacCodecCtx->sample_fmt,
                                                mAacCodecCtx->sample_rate,
                                                0, NULL);
            swr_init(au_convert_ctx);

            int out_linesize;
            int out_buffer_size=av_samples_get_buffer_size(&out_linesize,
                                                 mAacFrame->channels,
                                                 mAacFrame->nb_samples,
                                                 AV_SAMPLE_FMT_S16, 1);
            uint8_t *out_buffer=(uint8_t *)av_malloc(out_buffer_size);
            swr_convert(au_convert_ctx, &out_buffer, out_linesize,
                    (const uint8_t **)mAacFrame->data , mAacFrame->nb_samples);
            swr_free(&au_convert_ctx);
            au_convert_ctx = NULL;
            memcpy(outBuf,  out_buffer,  out_linesize);
            av_free(out_buffer);


            ret = out_linesize;
            #endif
        }else{
            av_strerror(result, errorbuf, sizeof(errorbuf));
        }
        av_frame_free(&mAacFrame);
    }

    return ret;
}


void DecoderAAC::releaseAACDecoder()
{
    if(mAacCodecCtx) {
        avcodec_close(mAacCodecCtx);
        avcodec_free_context(&mAacCodecCtx);
        mAacCodecCtx = NULL;
    }

    if(mAacFrame) {
        av_frame_free(&mAacFrame);
        mAacFrame = NULL;
    }
}
}
