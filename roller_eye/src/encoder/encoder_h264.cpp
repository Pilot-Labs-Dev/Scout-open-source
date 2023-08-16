#include <iostream>
#include"plt_assert.h"
#include"plt_tools.h"
#include "encoder_h264.h"
#include <plt_malloc.h>
// #include "roller_eye/graphic_utils.h"

using namespace std;

namespace roller_eye
{

#if 0
static void printX264HeadersInfo(x264_t *pHandle)
{
    x264_nal_t* pNals;
    int iNal = 0;
    x264_encoder_headers(pHandle, &pNals, &iNal);
    for (int i = 0; i < iNal; ++i)
    {
        switch (pNals[i].i_type)
        {
        case NAL_SPS:
            printf("SPS size: %d\n", pNals[i].i_payload);
            for (int j = 0; j < pNals[i].i_payload; j++)
                printf("0x%0x, ", *(pNals[i].p_payload + j));
            printf("\n");
        break;
        case NAL_PPS:
            printf("PPS size: %d\n", pNals[i].i_payload);
            for (int j = 0; j < pNals[i].i_payload; j++)
                printf("0x%0x, ", *(pNals[i].p_payload + j));
            printf("\n");
        break;
        case NAL_SEI:
            printf("SEI size: %d\n", pNals[i].i_payload);
            break;
        default:
            printf("type: %d, size: %d\n", pNals[i].i_type, pNals[i].i_payload);
        break;
        }
    }
}
#endif

EncoderH264::EncoderH264(StreamInfo *sInfo)
{
    int csp = X264_CSP_I420;
    int width= sInfo->fInfo.width;
    int height= sInfo->fInfo.height;
    int fps = sInfo->fpsNum / sInfo->fpsDen;
    if (fps == 0) {
        fps = sInfo->fpsDen / sInfo->fpsNum;
    }
    fmt=sInfo->fInfo.fmt;
    plt_assert(fmt==V4L2_PIX_FMT_YUV422P||fmt==V4L2_PIX_FMT_YUYV||fmt==V4L2_PIX_FMT_YUV420);

    pNals = nullptr;
    pHandle = nullptr;
    pPic_in = (x264_picture_t*)plt_malloc(sizeof(x264_picture_t));
    pPic_out = (x264_picture_t*)plt_malloc(sizeof(x264_picture_t));
    pParam = (x264_param_t*)plt_malloc(sizeof(x264_param_t));

    pPic_in->img.plane[0] = nullptr;
    pPic_in->img.plane[1] = nullptr;
    pPic_in->img.plane[2] = nullptr;

    // x264_param_default(pParam);
    if(x264_param_default_preset(pParam, "veryfast", "zerolatency") < 0){
        cout << "x264_param_default_preset failed." << endl;
    }
    pParam->i_csp = csp;
    pParam->i_width   = width;
    pParam->i_height  = height;

    pParam->i_threads  = 1;
    pParam->i_slice_count = 1;
    pParam->i_slice_count_max = 1;
    pParam->b_repeat_headers = 1;
    pParam->i_fps_num  = fps;
    pParam->i_fps_den  = 1;
    pParam->i_timebase_num = pParam->i_fps_den;
    pParam->i_timebase_den = pParam->i_fps_num;
    pParam->i_keyint_max = fps;
    //pParam->i_keyint_min = fps;
    //pParam->b_intra_refresh = 0;
    //pParam->b_sliced_threads = false;
    //pParam->b_annexb = 1;

    pParam->i_log_level  = X264_LOG_WARNING;

    /*
    //Param
    pParam->i_frame_total = 0;
    pParam->i_bframe  = 5;
    pParam->b_open_gop  = 0;
    pParam->i_bframe_pyramid = 0;
    pParam->rc.i_qp_constant=0;
    pParam->rc.i_qp_max=0;
    pParam->rc.i_qp_min=0;
    pParam->i_bframe_adaptive = X264_B_ADAPT_TRELLIS;
    */
    x264_param_apply_profile(pParam, x264_profile_names[4]);

    pHandle = x264_encoder_open(pParam);

    x264_picture_init(pPic_out);
    if (0 != x264_picture_alloc(pPic_in, csp, pParam->i_width, pParam->i_height)) {
        cout << "x264_picture_alloc failed." << endl;
    }

    pPic_in->i_pts = 0;
}

EncoderH264::~EncoderH264()
{
    int ret;
    int iNal = 0;

    while(1){
        ret = x264_encoder_encode(pHandle, &pNals, &iNal, NULL, pPic_out);
        if(ret==0){
            break;
        }
    }
    x264_picture_clean(pPic_in);
    x264_encoder_close(pHandle);
    pHandle = nullptr;

    plt_free(pPic_in);
    plt_free(pPic_out);
    plt_free(pParam);
}

int EncoderH264::put_frame(const Frame *frame)
{
    //cout<< "put_frame." << pParam->i_width * pParam->i_height << endl;
    if (frame->getSize() <= 0 || frame->getData() == nullptr) {
        //cout << "put frame error." << endl;
        return -1;
    }

    if (fmt == V4L2_PIX_FMT_YUYV)
	{
		yuyv2I420(frame->getData(),pPic_in->img.plane[0],pPic_in->img.plane[1],pPic_in->img.plane[2],pParam->i_width,pParam->i_height);
	}
	else if (fmt == V4L2_PIX_FMT_YUV422P )
	{
		I422toI420(frame->getData(),pPic_in->img.plane[0],pPic_in->img.plane[1],pPic_in->img.plane[2],pParam->i_width,pParam->i_height);
	}
    else if(fmt == V4L2_PIX_FMT_YUV420)
    {
        int ysize=pParam->i_width*pParam->i_height;
        memcpy(pPic_in->img.plane[0],frame->getData(),ysize);
        memcpy(pPic_in->img.plane[1],frame->getData()+ysize,ysize/4);
        memcpy(pPic_in->img.plane[2],frame->getData()+ysize+ysize/4,ysize/4);
    }

    return 0;
}

const Frame* EncoderH264::encode_frame()
{
    static int64_t pts = 0;
    int iNal = 0;

    pPic_in->i_pts = pts++;
    //pPic_in->i_type = X264_TYPE_KEYFRAME; //X264_TYPE_AUTO
    int frame_size = x264_encoder_encode(pHandle, &pNals, &iNal, pPic_in, pPic_out);
    if (frame_size <= 0){
        printf("Wait.");
        return nullptr;
    }
    // printf("h264 frame: %5ld, size: %5d, iNal: %d, keyframe: %d\n", pPic_in->i_pts, frame_size, iNal, pPic_out->b_keyframe);
    //printf("=-----%x, %x, %x, %x, %x \n", mFrame.getData()[0], mFrame.getData()[1], mFrame.getData()[2], mFrame.getData()[3], mFrame.getData()[4]);

    mFrame.setData(pNals->p_payload,frame_size);
    mFrame.isKeyframe = pPic_out->b_keyframe;

    return &mFrame;
}


} // namespace roller_eye
