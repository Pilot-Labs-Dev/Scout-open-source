#include"encoder.h"
#include "encoder_jpeg.h"
#include <iostream>

namespace roller_eye
{

EncoderJPEG::EncoderJPEG(StreamInfo *sInfo) : mOutBuf(nullptr), mOutSize(0), mQuality(75)
{
    mFmt = sInfo->fInfo.fmt;
    mWidth = sInfo->fInfo.width;
    mHeight = sInfo->fInfo.height;
    // printf("EncoderJPEG, mFmt: 0x%x\n", mFmt);
}

EncoderJPEG::~EncoderJPEG()
{
    if (mOutBuf != nullptr) {
        free(mOutBuf);
        mOutBuf = nullptr;
    }
}


void EncoderJPEG::my_error_exit(j_common_ptr cinfo)
{
    /* cinfo->err really points to a my_error_mgr struct, so coerce pointer */
    my_error_mgr* myerr = (my_error_mgr*) cinfo->err;
    (*cinfo->err->output_message) (cinfo);
    longjmp(myerr->setjmp_buf, 1);
}

int EncoderJPEG::put_frame(const Frame *frame)
{
    if (frame->getSize() <= 0 || frame->getData() == nullptr) {
        //std::cout << "put frame error." << std::endl;
        return -1;
    }

    const uint8_t *frame_buffer = frame->getData();
    int quality = mQuality;

    int row_stride = 0;
    JSAMPROW row_pointer[1];

    cinfo.err = jpeg_std_error(&jerr.pub);
    jerr.pub.error_exit = my_error_exit;

    if (setjmp(jerr.setjmp_buf)) {
        jpeg_destroy_compress(&cinfo);
        //fclose(fp);
        return -1;
    }

    jpeg_create_compress(&cinfo);
    // fp = fopen("/tmp/jpeg_file.jpg", "wb");
    // if (fp == NULL) {
    //     printf("open file failed.\n");
    //     return -1;
    // }
    // jpeg_stdio_dest(&cinfo, fp);
    if (mOutBuf != nullptr) {
        free(mOutBuf);
        mOutBuf = nullptr;
    }
    jpeg_mem_dest(&cinfo, &mOutBuf, &mOutSize);

    cinfo.image_width = mWidth;
    cinfo.image_height = mHeight;
    cinfo.input_components = 3;
    cinfo.in_color_space = JCS_YCbCr;

    jpeg_set_defaults(&cinfo);
    jpeg_set_quality(&cinfo, quality, TRUE);
    jpeg_start_compress(&cinfo, TRUE);

    row_stride = cinfo.image_width * cinfo.input_components;
    uint8_t *row_buf = new uint8_t[row_stride];
    row_pointer[0] = row_buf;

    if (mFmt == V4L2_PIX_FMT_YUYV) {
        while (cinfo.next_scanline < cinfo.image_height) {
            unsigned i, j;
            unsigned offset = cinfo.next_scanline * cinfo.image_width * 2;
            for (i = 0, j = 0; i < cinfo.image_width*2; i += 4, j += 6) {
                row_buf[j + 0] = frame_buffer[offset + i + 0]; // Y
                row_buf[j + 1] = frame_buffer[offset + i + 1]; // U
                row_buf[j + 2] = frame_buffer[offset + i + 3]; // V
                row_buf[j + 3] = frame_buffer[offset + i + 2]; // Y
                row_buf[j + 4] = frame_buffer[offset + i + 1]; // U
                row_buf[j + 5] = frame_buffer[offset + i + 3]; // V
            }
            //row_pointer[0] = (unsigned char*)&frame_buffer[cinfo.next_scanline * row_stride];
            jpeg_write_scanlines(&cinfo, row_pointer, 1);
        }
    } else if (mFmt == V4L2_PIX_FMT_YUV420) {
        const uint8_t *ybase, *ubase, *vbase;
        ybase = frame_buffer;
        ubase = frame_buffer + cinfo.image_width * cinfo.image_height;
        vbase = ubase + (cinfo.image_width * cinfo.image_height >>2);
        int j = 0;
        while (cinfo.next_scanline < cinfo.image_height) {
            int idx = 0;
            for(unsigned i = 0; i < cinfo.image_width; i++) {
                row_buf[idx++] = ybase[j * cinfo.image_width + i];
                row_buf[idx++] = ubase[(j>>1) * (cinfo.image_width >>1) + (i>>1)];
                row_buf[idx++] = vbase[(j >>1)* (cinfo.image_width >>1) + (i>>1)];
            }
            jpeg_write_scanlines(&cinfo, row_pointer, 1);
            j++;
        }
    } else {
        printf("NOT define fmt: 0x%x", mFmt);;
    }

    delete[] row_buf;
    return 0;
}

const Frame* EncoderJPEG::encode_frame()
{
    jpeg_finish_compress(&cinfo);
    jpeg_destroy_compress(&cinfo);
    // fclose(fp);
    // exit(0);

    mFrame.setData(mOutBuf, mOutSize);
    return &mFrame;
}

} // namespace roller_eye

