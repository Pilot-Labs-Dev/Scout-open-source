#include"encoder.h"
#include "encoder_aac.h"
#include <faac.h>
#include <string.h>
#include"plog_plus.h"


namespace roller_eye
{
#define AAC_ENCODER_TAG   "AACEncoder"

#if 0
static void printDecoderSpec(faacEncHandle handle)
{
    uint8_t *dsinfo = NULL;
    uint64_t dslen = 0;
    if (0 == faacEncGetDecoderSpecificInfo(hd, &dsinfo, &dslen)) {
        for (int i = 0; i < (int)dslen; i++)
            printf("dsinfo[%d]: 0x%x\n", i, dsinfo[i]);
        free(dsinfo);
    }
}
#endif

EncoderAAC::EncoderAAC(int rate, int bitwidth, int channels)
{
    m_bitWidth = bitwidth;
    m_rate = rate;
    m_channels = channels;

    if (aac_init(rate, bitwidth, channels) < 0)
        PLOG_ERROR(AAC_ENCODER_TAG,"\nInit aac encoder failed!\n");
}

EncoderAAC::~EncoderAAC()
{
    delete[] m_aacData;
    faacEncClose(m_aacEncHandle);
}

int EncoderAAC::put_frame(const Frame *frame)
{
    if (frame->getData() == nullptr || frame->getSize() <= 0) {
        return -1;
    }
    m_pcmData = const_cast<uint8_t*>(frame->getData());
    m_pcmSize = frame->getSize();

    return 0;
}

const Frame* EncoderAAC::encode_frame()
{
    m_aacSize = aac_enc_pcm(m_pcmData, m_pcmSize, m_aacData);
    if (m_aacSize <= 0) {
        return nullptr;
    }

    mFrame.setData(m_aacData,m_aacSize);
    mFrame.sampleRate = m_rate;
    mFrame.bitwidth = m_bitWidth;
    mFrame.channels = m_channels;

    return &mFrame;
}

int EncoderAAC::aac_init(int rate, int bitwidth, int channels)
{
    m_aacEncHandle = faacEncOpen(rate, channels, &m_inputSamples, &m_maxoutbytes);
    if (m_aacEncHandle == nullptr) {
        PLOG_ERROR(AAC_ENCODER_TAG,"Failed to open faac enc!\n");
        return -1;
    }
    m_aacData = new uint8_t[m_maxoutbytes];
    m_configPtr = faacEncGetCurrentConfiguration(m_aacEncHandle);
    m_configPtr->inputFormat = FAAC_INPUT_16BIT;
    m_configPtr->outputFormat = 1;
    // m_configPtr->useTns = 1;
    m_configPtr->useLfe = 0;
    m_configPtr->aacObjectType = LOW;
    m_configPtr->allowMidside = 0;
    //m_configPtr->shortctl=SHORTCTL_NORMAL;
    //m_configPtr->bandWidth = 32000;
    //m_configPtr->bitRate = 48000;

    if (faacEncSetConfiguration(m_aacEncHandle, m_configPtr) < 0) {
        PLOG_ERROR(AAC_ENCODER_TAG,"Cannot set configure for aac handle!\n");
        return -1;
    }

    return 0;
}

int EncoderAAC::aac_enc_pcm(unsigned char *pcmbuffer, unsigned int buflen, unsigned char *out)
{
    //unsigned int inputsample = buflen /(m_bitWidth / 8);
    int ret = faacEncEncode(m_aacEncHandle, (int32_t *)pcmbuffer, m_inputSamples, out, m_maxoutbytes);
    if(ret < 1) {
        PLOG_ERROR(AAC_ENCODER_TAG,"aac encoder wait!\n");
        return -1;
    }
    // printf("enc pcmsize:%d, samples:%d, aacsize:%d\n", buflen, inputsample, ret);
    return ret;
}



} // namespace roller_eye

