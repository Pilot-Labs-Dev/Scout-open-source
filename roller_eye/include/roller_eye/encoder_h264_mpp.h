#ifndef __ENCODER_H264_MPP_H__
#define __ENCODER_H264_MPP_H__

#include "encoder.h"
#include "video_stream.h"
#include "rk_mpi.h"
#include "osal/mpp_env.h"
#include "osal/mpp_mem.h"
#include "osal/mpp_log.h"
#include "osal/mpp_time.h"
#include "osal/mpp_common.h"

namespace roller_eye
{
#define MPI_DEBUG 0

typedef struct {
    // global flow control flag
    RK_U32 frm_eos;
    RK_U32 pkt_eos;
    RK_U32 frame_count;
    RK_U64 stream_size;

    // base flow context
    MppCtx ctx;
    MppApi *mpi;
    MppEncPrepCfg prep_cfg;
    MppEncRcCfg rc_cfg;
    MppEncCodecCfg codec_cfg;

    // input / output
    MppBuffer frm_buf;
    MppEncSeiMode sei_mode;

    // paramter for resource malloc
    RK_U32 width;
    RK_U32 height;
    RK_U32 hor_stride;
    RK_U32 ver_stride;
    MppFrameFormat fmt;
    MppCodingType type;
    RK_U32 num_frames;

    // resources
    size_t frame_size;
    /* NOTE: packet buffer may overflow */
    size_t packet_size;

    // rate control runtime parameter
    RK_S32 gop;
    RK_S32 fps;
    RK_S32 bps;

    // sps&pps
    RK_U8 *exinfo;
    RK_S32 exinfo_len;
} MpiEncData;


class EncoderH264MPP : public Encoder {
    public:
    EncoderH264MPP(StreamInfo *fInfo);
    ~EncoderH264MPP();
    int put_frame(const Frame *frame);
    const Frame* encode_frame();

    private:
    MPP_RET mpp_ctx_init(MpiEncData **data);
    MPP_RET mpp_ctx_deinit(MpiEncData **data);
    void getExtraInfo(MpiEncData *p);
    MPP_RET mpp_setup(MpiEncData *p, int fps, int quality);
    MPP_RET read_yuv_image(RK_U8 *buf, RK_U32 width, RK_U32 height,
        RK_U32 hor_stride, RK_U32 ver_stride, MppFrameFormat fmt, const uint8_t *frameData);

    MppCodingType   mType;
    RK_U32          mWidth;
    RK_U32          mHeight;
    MppFrameFormat  mFormat;

    MpiEncData *mMpp;
    VideoFrame mFrame;
};

} // namespace roller_eye

#endif // __ENCODER_H264_MPP_H__
