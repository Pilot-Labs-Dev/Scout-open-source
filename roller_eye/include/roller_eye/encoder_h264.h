#ifndef __ENCODER_H264_H__
#define __ENCODER_H264_H__

#include"encoder.h"
#include "video_stream.h"
#include <x264.h>

namespace roller_eye
{

class EncoderH264 : public Encoder {
    public:
    EncoderH264(StreamInfo *fInfo);
    ~EncoderH264();
    int put_frame(const Frame *frame);
    const Frame* encode_frame();
    //for test
    //void draw_rect(int rect_x, int rect_y, int rect_w, int rect_h, int R, int G, int B);

    private:
    int fmt;
    x264_nal_t* pNals;
    x264_t* pHandle;
    x264_picture_t* pPic_in;
    x264_picture_t* pPic_out;
    x264_param_t* pParam;
    VideoFrame mFrame;
};
} // namespace roller_eye

#endif // __ENCODER_H264_H__
