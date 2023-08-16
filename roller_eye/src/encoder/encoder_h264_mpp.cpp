#include <iostream>
#include "encoder_h264_mpp.h"
#include <string.h>

using namespace std;

namespace roller_eye
{


EncoderH264MPP::EncoderH264MPP(StreamInfo *sInfo) {
    //cout<< "EncoderH264MPP" << endl;
    mType = MPP_VIDEO_CodingAVC;
    mWidth = sInfo->fInfo.width;
    mHeight = sInfo->fInfo.height;
    int fps = sInfo->fpsNum / sInfo->fpsDen;
    if (fps == 0) {
        fps = sInfo->fpsDen / sInfo->fpsNum;
    }

    if (sInfo->fInfo.fmt == V4L2_PIX_FMT_YUYV) {
        mFormat = MPP_FMT_YUV422_YUYV;
    } else if (sInfo->fInfo.fmt == V4L2_PIX_FMT_YUV420) {
        mFormat = MPP_FMT_YUV420P;
    } else {
        //printf("NOT define fmt: 0x%x", sInfo->fInfo.fmt);
        exit(-1);
    }

    mpp_env_set_u32("mpi_debug", MPI_DEBUG);
    MPP_RET ret = MPP_OK;

    ret = mpp_ctx_init(&mMpp);
    if (ret) {
        mpp_err_f("mpp_ctx_init failed ret %d\n", ret);
        goto MPP_INIT_OUT;
    }

    ret = mpp_buffer_get(NULL, &mMpp->frm_buf, mMpp->frame_size);
    if (ret) {
        mpp_err_f("failed to get buffer for input frame ret %d\n", ret);
        goto MPP_INIT_OUT;
    }

    ret = mpp_create(&mMpp->ctx, &mMpp->mpi);
    if (ret) {
        mpp_err("mpp_create failed ret %d\n", ret);
        goto MPP_INIT_OUT;
    }

    ret = mpp_init(mMpp->ctx, MPP_CTX_ENC, mMpp->type);
    if (ret) {
        mpp_err("mpp_init failed ret %d\n", ret);
        goto MPP_INIT_OUT;
    }

    mpp_log_f("mpp_setup failed sInfo->h264Quality %d\n", sInfo->h264Quality);
    ret = mpp_setup(mMpp, fps, sInfo->h264Quality);
    if (ret) {
        mpp_err_f("mpp_setup failed ret %d\n", ret);
        goto MPP_INIT_OUT;
    }

    getExtraInfo(mMpp);
    return;

MPP_INIT_OUT:
    if (mMpp->ctx) {
        mpp_destroy(mMpp->ctx);
        mMpp->ctx = NULL;
    }

    if (mMpp->frm_buf) {
        mpp_buffer_put(mMpp->frm_buf);
        mMpp->frm_buf = NULL;
    }

    mpp_ctx_deinit(&mMpp);

    mpp_env_set_u32("mpi_debug", 0x0);
}

EncoderH264MPP::~EncoderH264MPP() {
    //cout<< "~EncoderH264MPP" << endl;
    MPP_RET ret = MPP_OK;

    ret = mMpp->mpi->reset(mMpp->ctx);
    if (ret) {
        mpp_err("mpi->reset failed\n");
    }

    if (mMpp->ctx) {
        mpp_destroy(mMpp->ctx);
        mMpp->ctx = NULL;
    }

    if (mMpp->frm_buf) {
        mpp_buffer_put(mMpp->frm_buf);
        mMpp->frm_buf = NULL;
    }

    if (mMpp->exinfo) {
        free(mMpp->exinfo);
        mMpp->exinfo = NULL;
        mMpp->exinfo_len = 0;
    }

    mpp_log("successful total frame %d bps %lld\n",
            mMpp->frame_count, (RK_U64)((mMpp->stream_size * 8 * mMpp->fps) / mMpp->frame_count));

    mpp_ctx_deinit(&mMpp);
    mpp_env_set_u32("mpi_debug", 0x0);
}

MPP_RET EncoderH264MPP::read_yuv_image(RK_U8 *buf, RK_U32 width, RK_U32 height,
                       RK_U32 hor_stride, RK_U32 ver_stride, MppFrameFormat fmt, const uint8_t *frameData)
{
    MPP_RET ret = MPP_OK;
    RK_U32 row = 0;
    RK_U8 *buf_y = buf;
    RK_U8 *buf_u = buf_y + hor_stride * ver_stride; // NOTE: diff from gen_yuv_image
    RK_U8 *buf_v = buf_u + hor_stride * ver_stride / 4; // NOTE: diff from gen_yuv_image

    switch (fmt) {
        case MPP_FMT_YUV420P : {
            for (row = 0; row < height; row++) {
                memcpy(buf_y + row * hor_stride, frameData + row * width, width);
            }
            for (row = 0; row < height / 2; row++) {
                memcpy(buf_u + row * hor_stride / 2, frameData + width*height + row * width/2, width/2);
            }
            for (row = 0; row < height / 2; row++) {
                memcpy(buf_v + row * hor_stride / 2, frameData + width*height*5/4 + row * width/2, width/2);
            }
        } break;

        case MPP_FMT_YUV422_YUYV :
        case MPP_FMT_YUV422_UYVY : {
            for (row = 0; row < height; row++) {
                memcpy(buf_y + row * hor_stride, frameData, width * 2);
                frameData += width * 2;
            }
        } break;

        default : {
            mpp_err_f("read image do not support fmt %d\n", fmt);
            ret = MPP_ERR_VALUE;
        } break;
    }

    return ret;
}

int EncoderH264MPP::put_frame(const Frame *inFrame)
{
    //cout<< "put_frame." << pParam->i_width * pParam->i_height << endl;
    if (inFrame->getSize() <= 0 || inFrame->getData() == nullptr) {
        return -1;
    }

    MPP_RET ret;
    MppApi *mpi = mMpp->mpi;
    MppCtx ctx = mMpp->ctx;

    MppFrame frame = NULL;
    void *buf = mpp_buffer_get_ptr(mMpp->frm_buf);

    ret = read_yuv_image((RK_U8*)buf, mMpp->width, mMpp->height,
                    mMpp->hor_stride, mMpp->ver_stride, mMpp->fmt, inFrame->getData());
    if (ret == MPP_NOK || ret == MPP_ERR_VALUE) {
        mpp_log("read yuv failed.\n");
        //mMpp->frm_eos = 1;
        return -1;
    }

    ret = mpp_frame_init(&frame);
    if (ret) {
        mpp_err_f("mpp_frame_init failed\n");
        return -1;
    }

    mpp_frame_set_width(frame, mMpp->width);
    mpp_frame_set_height(frame, mMpp->height);
    mpp_frame_set_hor_stride(frame, mMpp->hor_stride);
    mpp_frame_set_ver_stride(frame, mMpp->ver_stride);
    mpp_frame_set_fmt(frame, mMpp->fmt);
    mpp_frame_set_eos(frame, mMpp->frm_eos);

    mpp_frame_set_buffer(frame, mMpp->frm_buf);

    ret = mpi->encode_put_frame(ctx, frame);
    if (ret) {
        mpp_err("mpp encode put frame failed\n");
        return -1;
    }

    return 0;
}

const Frame* EncoderH264MPP::encode_frame()
{
    int is_keyframe = 0;
    MPP_RET ret;
    MppApi *mpi = mMpp->mpi;
    MppCtx ctx = mMpp->ctx;
    MppPacket packet = NULL;

    ret = mpi->encode_get_packet(ctx, &packet);
    if (ret || !packet) {
        mpp_err("mpp encode get packet failed\n");
        return nullptr;
    }

    RK_U8 *ptr  = (RK_U8*)mpp_packet_get_pos(packet);
    size_t len  = mpp_packet_get_length(packet);
    if (len<=0 || !ptr){
        mpp_err("mpp_packet_get_length failed: %d\n", len);
    }

    mMpp->pkt_eos = mpp_packet_get_eos(packet);
    ret = mpp_packet_deinit(&packet);
    if (ret){
        mpp_err("mpp_packet_deinit failed: %d\n", ret);
    }

    //mpp_log("encoded frame %d size %d\n", mMpp->frame_count, len);
    mMpp->stream_size += len;
    mMpp->frame_count++;

    if (mMpp->pkt_eos) {
        mpp_log("found last packet\n");
        mpp_assert(mMpp->frm_eos);
    }

    if (ptr[4] == 0x25) {
        is_keyframe = 1;
        RK_U8* tmp = (RK_U8*)realloc(mMpp->exinfo, mMpp->exinfo_len + len);
        if (tmp == NULL) {
            mpp_err("realloc exinfo failed!");
            return nullptr;
        }
        mMpp->exinfo = tmp;
        memcpy(mMpp->exinfo + mMpp->exinfo_len, ptr, len);
        mFrame.setData(mMpp->exinfo,mMpp->exinfo_len + len);
    } else {
        mFrame.setData(ptr,len);
    }

    // mpp_log("%x, %x, %x, %x, %x \n", mFrame.getData()[0], mFrame.getData()[1], mFrame.getData()[2], mFrame.getData()[3], mFrame.getData()[4]);
    mFrame.isKeyframe = is_keyframe;

    return &mFrame;
}


MPP_RET EncoderH264MPP::mpp_ctx_init(MpiEncData **data)
{
    MpiEncData *p = NULL;
    MPP_RET ret = MPP_OK;

    if (!data) {
        mpp_err_f("invalid input data %p \n", data);
        return MPP_ERR_NULL_PTR;
    }

    p = mpp_calloc(MpiEncData, 1);
    if (!p) {
        mpp_err_f("create MpiEncData failed\n");
        ret = MPP_ERR_MALLOC;
        goto RET;
    }

    // get paramter from cmd
    p->width        = mWidth;
    p->height       = mHeight;
    p->hor_stride   = MPP_ALIGN(mWidth, 16);
    p->ver_stride   = MPP_ALIGN(mHeight, 16);
    p->fmt          = mFormat;
    p->type         = mType;

    if (p->type == MPP_VIDEO_CodingMJPEG)
        p->num_frames = 1;
    //p->num_frames   = cmd->num_frames;

    // update resource parameter
    if (p->fmt <= MPP_FMT_YUV420SP_VU)
        p->frame_size = p->hor_stride * p->ver_stride * 3 / 2;
    else if (p->fmt <= MPP_FMT_YUV422_UYVY) {
        // NOTE: yuyv and uyvy need to double stride
        p->hor_stride *= 2;
        p->frame_size = p->hor_stride * p->ver_stride;
    } else
        p->frame_size = p->hor_stride * p->ver_stride * 4;

    p->packet_size  = p->width * p->height;

RET:
    *data = p;
    return ret;
}

MPP_RET EncoderH264MPP::mpp_ctx_deinit(MpiEncData **data)
{
    MpiEncData *p = NULL;

    if (!data) {
        mpp_err_f("invalid input data %p\n", data);
        return MPP_ERR_NULL_PTR;
    }

    p = *data;
    if (p) {
        MPP_FREE(p);
        *data = NULL;
    }

    return MPP_OK;
}

MPP_RET EncoderH264MPP::mpp_setup(MpiEncData *p, int fps, int quality)
{
    MPP_RET ret;
    MppApi *mpi;
    MppCtx ctx;
    MppEncCodecCfg *codec_cfg;
    MppEncPrepCfg *prep_cfg;
    MppEncRcCfg *rc_cfg;

    if (NULL == p)
        return MPP_ERR_NULL_PTR;

    switch (quality)
    {
    case 0:  //low
        quality = 64;
        break;
    case 1:  //middle
        quality = 32;
        break;
    case 2: //high
        quality = 16;
        break;
    default:
        quality = 32;
        break;
    }
    mpi = p->mpi;
    ctx = p->ctx;
    codec_cfg = &p->codec_cfg;
    prep_cfg = &p->prep_cfg;
    rc_cfg = &p->rc_cfg;

    /* setup default parameter */
    p->fps = fps;
    p->gop = fps;
    //p->bps = p->width * p->height / 16 * p->fps;
    p->bps = p->width * p->height / (quality )* p->fps;

    prep_cfg->change        = MPP_ENC_PREP_CFG_CHANGE_INPUT |
                              MPP_ENC_PREP_CFG_CHANGE_ROTATION |
                              MPP_ENC_PREP_CFG_CHANGE_FORMAT;
    prep_cfg->width         = p->width;
    prep_cfg->height        = p->height;
    prep_cfg->hor_stride    = p->hor_stride;
    prep_cfg->ver_stride    = p->ver_stride;
    prep_cfg->format        = p->fmt;
    prep_cfg->rotation      = MPP_ENC_ROT_0;
    ret = mpi->control(ctx, MPP_ENC_SET_PREP_CFG, prep_cfg);
    if (ret) {
        mpp_err("mpi control enc set prep cfg failed ret %d\n", ret);
        goto RET;
    }

    rc_cfg->change  = MPP_ENC_RC_CFG_CHANGE_ALL;
    rc_cfg->rc_mode = MPP_ENC_RC_MODE_CBR;
    rc_cfg->quality = MPP_ENC_RC_QUALITY_MEDIUM;

    if (rc_cfg->rc_mode == MPP_ENC_RC_MODE_CBR) {
        /* constant bitrate has very small bps range of 1/16 bps */
        rc_cfg->bps_target   = p->bps;
        rc_cfg->bps_max      = p->bps * 17 / 16;
        rc_cfg->bps_min      = p->bps * 15 / 16;
    } else if (rc_cfg->rc_mode ==  MPP_ENC_RC_MODE_VBR) {
        if (rc_cfg->quality == MPP_ENC_RC_QUALITY_CQP) {
            /* constant QP does not have bps */
            rc_cfg->bps_target   = -1;
            rc_cfg->bps_max      = -1;
            rc_cfg->bps_min      = -1;
        } else {
            /* variable bitrate has large bps range */
            rc_cfg->bps_target   = p->bps;
            rc_cfg->bps_max      = p->bps * 17 / 16;
            rc_cfg->bps_min      = p->bps * 1 / 16;
        }
    }

    /* fix input / output frame rate */
    rc_cfg->fps_in_flex      = 0;
    rc_cfg->fps_in_num       = p->fps;
    rc_cfg->fps_in_denorm    = 1;
    rc_cfg->fps_out_flex     = 0;
    rc_cfg->fps_out_num      = p->fps;
    rc_cfg->fps_out_denorm   = 1;

    rc_cfg->gop              = p->gop;
    rc_cfg->skip_cnt         = 0;

    mpp_log("mpp_setup bps %d fps %d gop %d\n",
            rc_cfg->bps_target, rc_cfg->fps_out_num, rc_cfg->gop);
    ret = mpi->control(ctx, MPP_ENC_SET_RC_CFG, rc_cfg);
    if (ret) {
        mpp_err("mpi control enc set rc cfg failed ret %d\n", ret);
        goto RET;
    }

    codec_cfg->coding = p->type;
    switch (codec_cfg->coding) {
    case MPP_VIDEO_CodingAVC : {
        codec_cfg->h264.change = MPP_ENC_H264_CFG_CHANGE_PROFILE |
                                 MPP_ENC_H264_CFG_CHANGE_ENTROPY |
                                 MPP_ENC_H264_CFG_CHANGE_TRANS_8x8;
        /*
         * H.264 profile_idc parameter
         * 66  - Baseline profile
         * 77  - Main profile
         * 100 - High profile
         */
        codec_cfg->h264.profile  = 100;
        /*
         * H.264 level_idc parameter
         * 10 / 11 / 12 / 13    - qcif@15fps / cif@7.5fps / cif@15fps / cif@30fps
         * 20 / 21 / 22         - cif@30fps / half-D1@@25fps / D1@12.5fps
         * 30 / 31 / 32         - D1@25fps / 720p@30fps / 720p@60fps
         * 40 / 41 / 42         - 1080p@30fps / 1080p@30fps / 1080p@60fps
         * 50 / 51 / 52         - 4K@30fps
         */
        codec_cfg->h264.level    = 40;
        codec_cfg->h264.entropy_coding_mode  = 1;
        codec_cfg->h264.cabac_init_idc  = 0;
        codec_cfg->h264.transform8x8_mode = 1;
    } break;
    case MPP_VIDEO_CodingMJPEG : {
        codec_cfg->jpeg.change  = MPP_ENC_JPEG_CFG_CHANGE_QP;
        codec_cfg->jpeg.quant   = 10;
    } break;
    case MPP_VIDEO_CodingVP8 : {
    } break;
    case MPP_VIDEO_CodingHEVC : {
        codec_cfg->h265.change = MPP_ENC_H265_CFG_INTRA_QP_CHANGE;
        codec_cfg->h265.intra_qp = 26;
    } break;
    default : {
        mpp_err_f("support encoder coding type %d\n", codec_cfg->coding);
    } break;
    }
    ret = mpi->control(ctx, MPP_ENC_SET_CODEC_CFG, codec_cfg);
    if (ret) {
        mpp_err("mpi control enc set codec cfg failed ret %d\n", ret);
        goto RET;
    }

    /* optional */
    //p->sei_mode = MPP_ENC_SEI_MODE_ONE_FRAME;
    p->sei_mode = MPP_ENC_SEI_MODE_DISABLE;
    ret = mpi->control(ctx, MPP_ENC_SET_SEI_CFG, &p->sei_mode);
    if (ret) {
        mpp_err("mpi control enc set sei cfg failed ret %d\n", ret);
        goto RET;
    }

RET:
    return ret;
}

void EncoderH264MPP::getExtraInfo(MpiEncData *p)
{
    MPP_RET ret;
    MppApi *mpi = p->mpi;
    MppCtx ctx = p->ctx;

    if (p->type == MPP_VIDEO_CodingAVC) {
        MppPacket packet = NULL;
        ret = mpi->control(ctx, MPP_ENC_GET_EXTRA_INFO, &packet);
        if (ret) {
            mpp_err("mpi control enc get extra info failed\n");
            return;
        }

        /* get and write sps/pps for H.264 */
        if (packet) {
            void *ptr   = mpp_packet_get_pos(packet);
            size_t len  = mpp_packet_get_length(packet);

            mpp_log("get spspps len: %lu\n", len);
            //for (int i = 0; i < len; i++)
            //    printf("0x%x, ", ((uint8_t*)ptr)[i]);

            p->exinfo = (RK_U8 *)malloc(len);
            p->exinfo_len = len;
            memcpy(p->exinfo, ptr, len);

            packet = NULL;
        }
    } else {
        mpp_err("getExtraInfo not supported.\n");
    }
}

} // namespace roller_eye
