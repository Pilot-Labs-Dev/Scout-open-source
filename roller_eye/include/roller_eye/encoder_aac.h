#ifndef __ENCODER_AAC_H__
#define __ENCODER_AAC_H__

#include "encoder.h"
#include <faac.h>

namespace roller_eye
{

class EncoderAAC : public Encoder {
    public:
        EncoderAAC(int rate=16000, int bitwidth=16, int channels=1);
        ~EncoderAAC();
        int put_frame(const Frame *frame);
        const Frame* encode_frame();

    private:
        int aac_init(int rate, int bitwidth, int channels);
        int aac_enc_pcm(unsigned char *buff, unsigned int buflen, unsigned char *out);
        ulong m_inputSamples;
        ulong m_maxoutbytes;
        faacEncHandle m_aacEncHandle;
        faacEncConfigurationPtr m_configPtr;

        int m_rate, m_bitWidth, m_channels;
        int m_pcmSize;
        int m_aacSize;
        uint8_t* m_pcmData;
        uint8_t* m_aacData;
        AudioFrame mFrame;
};
} // namespace roller_eye

#endif // __ENCODER_AAC_H__
