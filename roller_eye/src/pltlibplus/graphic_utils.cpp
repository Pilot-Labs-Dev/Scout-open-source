#include <sys/time.h>
#include "graphic_utils.h"
#include <string.h>
#include<linux/videodev2.h>
#include<assert.h>
#include<plt_tools.h>
#include<plt_assert.h>


namespace roller_eye{

GraphicUtils::GraphicUtils()
{
#ifndef APP_ARCH_X86
    plt_assert(rkRga.RkRgaInit()==0);
#endif
}

void GraphicUtils::yuyv2rgb24(const unsigned char *src, unsigned char *dest, int width, int height, int stride)
{
	int j;

	while (--height >= 0) {
		for (j = 0; j + 1 < width; j += 2) {
			int u = src[1];
			int v = src[3];
			int u1 = (((u - 128) << 7) +  (u - 128)) >> 6;
			int rg = (((u - 128) << 1) +  (u - 128) +
					((v - 128) << 2) + ((v - 128) << 1)) >> 3;
			int v1 = (((v - 128) << 1) +  (v - 128)) >> 1;

			*dest++ = CLIP(src[0] + v1);
			*dest++ = CLIP(src[0] - rg);
			*dest++ = CLIP(src[0] + u1);

			*dest++ = CLIP(src[2] + v1);
			*dest++ = CLIP(src[2] - rg);
			*dest++ = CLIP(src[2] + u1);
			src += 4;
		}
		src += stride - (width * 2);
	}
}

#ifndef APP_ARCH_X86

static float get_bpp_from_format(int format)
{
    float bpp = 0;

    switch (format) {
        case RK_FORMAT_RGB_565:
            bpp = 2;
            break;
        case RK_FORMAT_RGB_888:
            bpp = 3;
            break;
        case RK_FORMAT_RGBA_8888:
            bpp = 4;
            break;
        case RK_FORMAT_RGBX_8888:
            bpp = 4;
            break;
        case RK_FORMAT_BGRA_8888:
            bpp = 4;
            break;
	    case RK_FORMAT_YCbCr_420_P:
	    case RK_FORMAT_YCrCb_420_SP:
            bpp = 1.5;
            break;
        //case RK_FORMAT_YCrCb_NV12:
            //bpp = 1.5;
            break;
    	//case RK_FORMAT_YCrCb_NV12_VIDEO:
            //bpp = 1.5;
            break;
        //case RK_FORMAT_YCrCb_NV12_10:
            //bpp = 1.875;
            break;
        default:
    	    printf("Is unsupport format now,please fix \n");
            return 0;
    }

    return bpp;
}

//rga: Rga err irq! INT[705],STATS[100]
void GraphicUtils::rgaConvert(uint8_t *srcData, int srcWidth, int srcHeight, int srcFormat,
    uint8_t *dstData, int dstWidth, int dstHeight, int dstFormat,int rotation)
{
    std::unique_lock<std::mutex> lock(mtx);
    int ret = -1;

    rga_info_t src;
    rga_info_t dst;

    memset(&src, 0, sizeof(rga_info_t));
    src.fd = -1;
    src.mmuFlag = 1;
    src.virAddr = srcData;

    memset(&dst, 0, sizeof(rga_info_t));
    dst.fd = -1;
    dst.mmuFlag = 1;
    dst.virAddr = dstData;

    /********** set the rect_info **********/
    rga_set_rect(&src.rect, 0,0,srcWidth,srcHeight,srcWidth/*stride*/,srcHeight,srcFormat);
    rga_set_rect(&dst.rect, 0,0,dstWidth,dstHeight,dstWidth/*stride*/,dstHeight,dstFormat);

    /************ set the rga_mod ,rotation\composition\scale\copy .... **********/
    src.blend = 0xFF0100;
    src.rotation=rotation;

    /********** call rga_Interface **********/
    // struct timeval tpend1, tpend2;
    // long usec1 = 0;
    // gettimeofday(&tpend1, NULL);
    ret = rkRga.RkRgaBlit(&src, &dst, NULL);
    if (ret) {
        //printf("RkRgaBlit error : %s\n", strerror(errno));
    }
    // gettimeofday(&tpend2, NULL);
    // usec1 = 1000 * (tpend2.tv_sec - tpend1.tv_sec) + (tpend2.tv_usec - tpend1.tv_usec) / 1000;
    // printf("cost_time=%ld ms\n", usec1);

}


void GraphicUtils::rgaDrmConvert(uint8_t *srcData, int srcWidth, int srcHeight, int srcFormat,
    uint8_t *dstData, int dstWidth, int dstHeight, int dstFormat)
{
    std::unique_lock<std::mutex> lock(mtx);
    int ret = -1;
	bo_t bo_src, bo_dst;

	/********** apply for src buffer and dst buffer **********/
	ret = rkRga.RkRgaGetAllocBuffer(&bo_src, srcWidth, srcHeight, get_bpp_from_format(srcFormat)*8);
	ret = rkRga.RkRgaGetAllocBuffer(&bo_dst, dstWidth, dstHeight, get_bpp_from_format(dstFormat)*8);

	/********** map buffer_address to userspace **********/
	rkRga.RkRgaGetMmap(&bo_src);
	rkRga.RkRgaGetMmap(&bo_dst);

    memcpy(bo_src.ptr, srcData, srcWidth*srcHeight*get_bpp_from_format(srcFormat));


    rga_info_t src;
    rga_info_t dst;

    memset(&src, 0, sizeof(rga_info_t));
    src.fd = -1;
    src.mmuFlag = 1;

    memset(&dst, 0, sizeof(rga_info_t));
    dst.fd = -1;
    dst.mmuFlag = 1;

    // long usec1 = 0;

    /********** get src_Fd **********/
    ret = rkRga.RkRgaGetBufferFd(&bo_src, &src.fd);
    // printf("src.fd =%d \n",src.fd);
    if (ret) {
        //printf("rgaGetsrcFd fail : %s\n", strerror(errno));
    }
    /********** get dst_Fd **********/
    ret = rkRga.RkRgaGetBufferFd(&bo_dst, &dst.fd);
    // printf("dst.fd =%d \n",dst.fd);
    if (ret) {
        //printf("rgaGetdstFd error : %s\n", strerror(errno));
    }
    /********** if not fd, try to check phyAddr and virAddr **************/
    if(src.fd <= 0|| dst.fd <= 0)
    {
    /********** check phyAddr and virAddr ,if none to get virAddr **********/
        if (( src.phyAddr != 0 || src.virAddr != 0 ) || src.hnd != 0 ){
            //ret = RkRgaGetHandleMapAddress( gbs->handle, &src.virAddr );
            //printf("src.virAddr =%p\n",src.virAddr);
            if(!src.virAddr){
                //printf("err! src has not fd and address for render ,Stop!\n");
                goto OUT;
            }
        }

        /********** check phyAddr and virAddr ,if none to get virAddr **********/
        if (( dst.phyAddr != 0 || dst.virAddr != 0 ) || dst.hnd != 0 ){
            //ret = RkRgaGetHandleMapAddress( gbd->handle, &dst.virAddr );
            //printf("dst.virAddr =%p\n",dst.virAddr);
            if(!dst.virAddr){
                //printf("err! dst has not fd and address for render ,Stop!\n");
                goto OUT;
            }
        }
    }

    /********** set the rect_info **********/
    rga_set_rect(&src.rect, 0,0,srcWidth,srcHeight,srcWidth/*stride*/,srcHeight,srcFormat);
    rga_set_rect(&dst.rect, 0,0,dstWidth,dstHeight,dstWidth/*stride*/,dstHeight,dstFormat);

    /************ set the rga_mod ,rotation\composition\scale\copy .... **********/
    //src.blend = 0xff0105;
    src.rotation = 0;
    //src.rotation = HAL_TRANSFORM_ROT_90;
    //src.rotation = HAL_TRANSFORM_ROT_180;
    //src.rotation = HAL_TRANSFORM_ROT_270;
    //src.rotation = HAL_TRANSFORM_FLIP_V;
    //src.rotation = HAL_TRANSFORM_FLIP_H;

    /********** call rga_Interface **********/
    // struct timeval tpend1, tpend2;
    // gettimeofday(&tpend1, NULL);
    ret = rkRga.RkRgaBlit(&src, &dst, NULL);
    if (ret) {
        //printf("RkRgaBlit error : %s\n", strerror(errno));
    }
    // gettimeofday(&tpend2, NULL);
    // usec1 = 1000 * (tpend2.tv_sec - tpend1.tv_sec) + (tpend2.tv_usec - tpend1.tv_usec) / 1000;
    // printf("cost_time=%ld ms\n", usec1);

    memcpy(dstData, bo_dst.ptr, dstWidth*dstHeight*get_bpp_from_format(dstFormat));

OUT:
	rkRga.RkRgaUnmap(&bo_src);
	rkRga.RkRgaUnmap(&bo_dst);

	ret = rkRga.RkRgaFree(&bo_src);
    if (ret) {
        //printf("RkRgaFree error1 : %d\n", ret);
    }
	ret = rkRga.RkRgaFree(&bo_dst);
    if (ret) {
        //printf("RkRgaFree error2 : %d\n", ret);
    }

    if (bo_src.fd > 0)
        close(bo_src.fd);
    if (bo_dst.fd > 0)
        close(bo_dst.fd);

    if (src.fd > 0)
        close(src.fd);
    if (dst.fd > 0)
        close(dst.fd);
}


void GraphicUtils::rgaDrmBlend(uint8_t *srcData, int srcWidth, int srcHeight, int srcFormat,
    uint8_t *dstData, int dstWidth, int dstHeight, int dstFormat)
{
    std::unique_lock<std::mutex> lock(mtx);
    int ret = -1;
	bo_t bo_src, bo_dst;

	/********** apply for src buffer and dst buffer **********/
	ret = rkRga.RkRgaGetAllocBuffer(&bo_src, srcWidth, srcHeight, get_bpp_from_format(srcFormat)*8);
	ret = rkRga.RkRgaGetAllocBuffer(&bo_dst, dstWidth, dstHeight, get_bpp_from_format(dstFormat)*8);

	/********** map buffer_address to userspace **********/
	rkRga.RkRgaGetMmap(&bo_src);
	rkRga.RkRgaGetMmap(&bo_dst);

    memcpy(bo_src.ptr, srcData, srcWidth*srcHeight*get_bpp_from_format(srcFormat));


    rga_info_t src;
    rga_info_t dst;

    memset(&src, 0, sizeof(rga_info_t));
    src.fd = -1;
    src.mmuFlag = 1;

    memset(&dst, 0, sizeof(rga_info_t));
    dst.fd = -1;
    dst.mmuFlag = 1;

    // long usec1 = 0;

    /********** get src_Fd **********/
    ret = rkRga.RkRgaGetBufferFd(&bo_src, &src.fd);
    // printf("src.fd =%d \n",src.fd);
    if (ret) {
        //printf("rgaGetsrcFd fail : %s\n", strerror(errno));
    }
    /********** get dst_Fd **********/
    ret = rkRga.RkRgaGetBufferFd(&bo_dst, &dst.fd);
    // printf("dst.fd =%d \n",dst.fd);
    if (ret) {
        //printf("rgaGetdstFd error : %s\n", strerror(errno));
    }
    /********** if not fd, try to check phyAddr and virAddr **************/
    if(src.fd <= 0|| dst.fd <= 0)
    {
    /********** check phyAddr and virAddr ,if none to get virAddr **********/
        if (( src.phyAddr != 0 || src.virAddr != 0 ) || src.hnd != 0 ){
            //ret = RkRgaGetHandleMapAddress( gbs->handle, &src.virAddr );
            //printf("src.virAddr =%p\n",src.virAddr);
            if(!src.virAddr){
                //printf("err! src has not fd and address for render ,Stop!\n");
                goto OUT;
            }
        }

        /********** check phyAddr and virAddr ,if none to get virAddr **********/
        if (( dst.phyAddr != 0 || dst.virAddr != 0 ) || dst.hnd != 0 ){
            //ret = RkRgaGetHandleMapAddress( gbd->handle, &dst.virAddr );
            //printf("dst.virAddr =%p\n",dst.virAddr);
            if(!dst.virAddr){
                //printf("err! dst has not fd and address for render ,Stop!\n");
                goto OUT;
            }
        }
    }

    /********** set the rect_info **********/
    rga_set_rect(&src.rect, 0,0,srcWidth,srcHeight,srcWidth/*stride*/,srcHeight,srcFormat);
    rga_set_rect(&dst.rect, 0,0,dstWidth,dstHeight,dstWidth/*stride*/,dstHeight,dstFormat);

    /************ set the rga_mod ,rotation\composition\scale\copy .... **********/
    src.blend = 0xff0405;


    /********** call rga_Interface **********/
    // struct timeval tpend1, tpend2;
    // gettimeofday(&tpend1, NULL);
    ret = rkRga.RkRgaBlit(&src, &dst, NULL);
    if (ret) {
        //printf("RkRgaBlit error : %s\n", strerror(errno));
    }
    // gettimeofday(&tpend2, NULL);
    // usec1 = 1000 * (tpend2.tv_sec - tpend1.tv_sec) + (tpend2.tv_usec - tpend1.tv_usec) / 1000;
    // printf("cost_time=%ld ms\n", usec1);

    memcpy(dstData, bo_dst.ptr, dstWidth*dstHeight*get_bpp_from_format(dstFormat));

OUT:
	rkRga.RkRgaUnmap(&bo_src);
	rkRga.RkRgaUnmap(&bo_dst);

	ret = rkRga.RkRgaFree(&bo_src);
    if (ret) {
        //printf("RkRgaFree error1 : %d\n", ret);
    }
	ret = rkRga.RkRgaFree(&bo_dst);
    if (ret) {
        //printf("RkRgaFree error2 : %d\n", ret);
    }

    if (bo_src.fd > 0)
        close(bo_src.fd);
    if (bo_dst.fd > 0)
        close(bo_dst.fd);

    if (src.fd > 0)
        close(src.fd);
    if (dst.fd > 0)
        close(dst.fd);
}

#endif // ifndef APP_ARCH_X86


void GraphicUtils::yuv420pBlend(unsigned char *frame1, int w1, int h1, unsigned char *frame2, int w2, int h2, int off_x, int off_y, unsigned char alpha)
{
    unsigned char *y1 = frame1;
    unsigned char *u1 = y1 + w1 * h1;
    unsigned char *v1 = u1 + w1 * h1 / 4;

    unsigned char *y2 = frame2;
    unsigned char *u2 = y2 + w2 * h2;
    unsigned char *v2 = u2 + w2 * h2 / 4;

    int i, j;
    int nOff = 0;
    for (i = 0; i < h2; i++) {
        nOff = w1 * (off_y + i) + off_x;
        for (j = 0; j < w2; j++) {
            *(y1 + nOff + j) = (*(y1 + nOff + j) * (255 - alpha) + *(y2 + w2 * i + j) * alpha) / 255;
        }
    }
    for (j = 0; j < h2 / 2; j++) {
        nOff = (w1 / 2) * (off_y / 2 + j) + off_x / 2;
        for (i = 0; i < w2 / 2; i++) {
            *(u1 + nOff + i) = (*(u1 + nOff + i) * (255 - alpha) + *(u2 + w2 / 2 * j + i) * alpha) / 255;
            *(v1 + nOff + i) = (*(v1 + nOff + i) * (255 - alpha) + *(v2 + w2 / 2 * j + i) * alpha) / 255;
        }
    }
}

void GraphicUtils::yuyv422Blend(unsigned char *frame1, int w1, int h1, unsigned char *frame2, int w2, int h2, int off_x, int off_y, unsigned char alpha)
{
    int i, j;
    int nOff = 0;
    for (i = 0; i < h2; i++) {
        nOff = w1*2 * (off_y + i) + off_x*2;
        for (j = 0; j < w2*2 - 4; j += 4) {
            *(frame1 + nOff + j + 0) = (*(frame1 + nOff + j + 0) * (255 - alpha) + *(frame2 + w2*2 * i + j + 0) * alpha) / 255;
            *(frame1 + nOff + j + 1) = (*(frame1 + nOff + j + 1) * (255 - alpha) + *(frame2 + w2*2 * i + j + 1) * alpha) / 255;
            *(frame1 + nOff + j + 2) = (*(frame1 + nOff + j + 2) * (255 - alpha) + *(frame2 + w2*2 * i + j + 2) * alpha) / 255;
            *(frame1 + nOff + j + 3) = (*(frame1 + nOff + j + 3) * (255 - alpha) + *(frame2 + w2*2 * i + j + 3) * alpha) / 255;
        }
    }
}

bool GraphicUtils::checkRectange(int w,int h,int left,int right,int top,int bottom)
{
    return (left >=0 && left < right && right<w && top>=0 && top<bottom && bottom < h);
}
void GraphicUtils::yuv420pDrawRectPlane(uint8_t *yp, uint8_t *up, uint8_t *vp, int pic_w, int rect_x, int rect_y, int rect_w, int rect_h, int R, int G, int B)
{
    int Y, U, V;
    Y =  0.299  * R + 0.587  * G + 0.114  * B;
    U = -0.1687 * R + 0.3313 * G + 0.5    * B + 128;
    V =  0.5    * R - 0.4187 * G - 0.0813 * B + 128;

    for (int i = rect_x; i < rect_x + rect_w; ++i) {
        if (i < pic_w) {
            yp[rect_y * pic_w + i]  = Y;
            up[rect_y/2 * pic_w/2 + i/2] = U;
            vp[rect_y/2 * pic_w/2 + i/2] = V;

            yp[(rect_y + rect_h) * pic_w + i]  = Y;
            up[(rect_y + rect_h)/2 * pic_w/2 + i/2] = U;
            vp[(rect_y + rect_h)/2 * pic_w/2 + i/2] = V;
        } else {
            break;
        }
    }

    for (int i = rect_y; i < rect_y + rect_h; ++i) {
        yp[i * pic_w + rect_x]  = Y;
        up[i/2 * pic_w/2 + rect_x/2] = U;
        vp[i/2 * pic_w/2 + rect_x/2] = V;

        if ((rect_x + rect_w) < pic_w) {
            yp[i * pic_w + rect_x + rect_w]  = Y;
            up[i/2 * pic_w/2 + rect_x/2 + rect_w/2] = U;
            vp[i/2 * pic_w/2 + rect_x/2 + rect_w/2] = V;
        }
    }
}

void GraphicUtils::yuv420pDrawRect(uint8_t *pic, int pic_w, int pic_h, int rect_x, int rect_y, int rect_w, int rect_h, int R, int G, int B)
{
    uint8_t *yp = pic;
    uint8_t *up = pic + pic_w * pic_h;
    uint8_t *vp = pic + pic_w * pic_h * 5/4;

	yuv420pDrawRectPlane(yp, up, vp, pic_w, rect_x, rect_y, rect_w, rect_h, R, G, B);
}

void GraphicUtils::yuyvDrawRect(uint8_t *pic, int pic_w, int rect_x, int rect_y, int rect_w, int rect_h, int R, int G, int B)
{
    int Y, U, V;
    Y =  0.299  * R + 0.587  * G + 0.114  * B;
    U = -0.1687 * R + 0.3313 * G + 0.5    * B + 128;
    V =  0.5    * R - 0.4187 * G - 0.0813 * B + 128;

    if (rect_x & 1) rect_x -= 1;
    if (rect_y & 1) rect_y -= 1;
    if (rect_w & 1) rect_w -= 1;
    if (rect_h & 1) rect_h -= 1;
    int DW = pic_w*2;

    for (int i = rect_x*2; i < (rect_x + rect_w)*2 - 4; i += 4) {
        if (i < DW - 4) {
            pic[rect_y * DW + i]  = Y;
            pic[rect_y * DW + i + 1]  = U;
            pic[rect_y * DW + i + 2]  = Y;
            pic[rect_y * DW + i + 3]  = V;

            pic[(rect_y + rect_h) * DW + i]  = Y;
            pic[(rect_y + rect_h) * DW + i + 1]  = U;
            pic[(rect_y + rect_h) * DW + i + 2]  = Y;
            pic[(rect_y + rect_h) * DW + i + 3]  = V;
        } else {
            break;
        }
    }

    for (int i = rect_y; i < rect_y + rect_h + 2; ++i) {
        pic[i * DW + rect_x*2]  = Y;
        pic[i * DW + rect_x*2 + 1]  = U;
        pic[i * DW + rect_x*2 + 2]  = Y;
        pic[i * DW + rect_x*2 + 3]  = V;

        if ((rect_x + rect_w) < pic_w) {
            pic[i * DW + rect_x*2 + rect_w*2 - 4]  = Y;
            pic[i * DW + rect_x*2 + rect_w*2 - 3]  = U;
            pic[i * DW + rect_x*2 + rect_w*2 - 2]  = Y;
            pic[i * DW + rect_x*2 + rect_w*2 - 1]  = V;
        }
    }
}
void GraphicUtils::decodeYData(const unsigned char *buff,unsigned char *y,int width,int height,int fmt)
{
    switch (fmt)
    {
    case V4L2_PIX_FMT_YUYV:
        copyYFromYUYV(buff,y,width,height);
        break;
    case V4L2_PIX_FMT_YUV422P:
    case V4L2_PIX_FMT_YUV420:
        memcpy(y,buff,width*height);
        break;
    default:
        assert(false);
        break;
    }
}
void GraphicUtils::decodeYDataAndTryScale(const unsigned char *buff,int width,int height,int fmt,vector<uint8_t> &dst,int &outW,int &outH)
{
#ifndef APP_ARCH_X86 //hardware accelator
    if(fmt==V4L2_PIX_FMT_YUV420){
        dst.resize(outW*outH*3/2);
        rgaConvert(const_cast<uint8_t*>(buff),width,height,RK_FORMAT_YCbCr_420_SP,dst.data(),outW,outH,RK_FORMAT_YCbCr_420_SP);
        dst.resize(outW*outH);
        return;
    }
#endif
    //not support hardware accelator,do not scale
    dst.resize(width*height);
    decodeYData(buff,dst.data(),width,height,fmt);
    outW=width;
    outH=height;
}
} // namespace roller_eye
