#ifndef __GRAPHIC_UTILS_H__
#define __GRAPHIC_UTILS_H__
#include<vector>
#include "roller_eye/single_class.h"
#ifndef APP_ARCH_X86
#include <RockchipRga.h>
#endif

namespace roller_eye {

#define CLIP(color) (unsigned char)(((color) > 0xFF) ? 0xff : (((color) < 0) ? 0 : (color)))


class GraphicUtils: public SingleClass<GraphicUtils>
{
public:
    static bool checkRectange(int w,int h,int left,int right,int top,int bottom);
    static void yuyv2rgb24(const unsigned char *src, unsigned char *dest, int width, int height, int stride);
    static void yuv420pDrawRectPlane(uint8_t *yp, uint8_t *up, uint8_t *vp, int pic_w, int rect_x, int rect_y, int rect_w, int rect_h, int R, int G, int B);
    static void yuv420pDrawRect(uint8_t *pic, int pic_w, int pic_h, int rect_x, int rect_y, int rect_w, int rect_h, int R, int G, int B);
    static void yuv420pBlend(unsigned char *frame1, int w1, int h1, unsigned char *frame2, int w2, int h2, int off_x, int off_y, unsigned char alpha);
    static void yuyvDrawRect(uint8_t *pic, int pic_w, int rect_x, int rect_y, int rect_w, int rect_h, int R, int G, int B);
    static void yuyv422Blend(unsigned char *frame1, int w1, int h1, unsigned char *frame2, int w2, int h2, int off_x, int off_y, unsigned char alpha);
    static void decodeYData(const unsigned char *buff,unsigned char *y,int width,int height,int fmt);

#ifndef APP_ARCH_X86
    void rgaConvert(uint8_t *srcData, int srcWidth, int srcHeight, int srcFormat, uint8_t *dstData, int dstWidth, int dstHeight, int dstFormat,int rotation=0);
    void rgaDrmConvert(uint8_t *srcData, int srcWidth, int srcHeight, int srcFormat, uint8_t *dstData, int dstWidth, int dstHeight, int dstFormat);
    void rgaDrmBlend(uint8_t *srcData, int srcWidth, int srcHeight, int srcFormat, uint8_t *dstData, int dstWidth, int dstHeight, int dstFormat);
#endif
    void decodeYDataAndTryScale(const unsigned char *buff,int width,int height,int fmt,vector<uint8_t> &dst,int &outW,int &outH);

private:
    friend class SingleClass<GraphicUtils>;
    GraphicUtils();
    GraphicUtils(const GraphicUtils &);
    GraphicUtils& operator=(const GraphicUtils &);

    std::mutex mtx;
#ifndef APP_ARCH_X86
    RockchipRga rkRga;
#endif
};

} // namespace roller_eye

#endif // __GRAPHIC_UTILS_H__
