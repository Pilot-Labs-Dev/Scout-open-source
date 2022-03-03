#include"encoder.h"
#include "video_stream.h"
#include <setjmp.h>
#include <jpeglib.h>
#include <jerror.h>

namespace roller_eye
{

struct my_error_mgr {
    struct jpeg_error_mgr pub;
    jmp_buf setjmp_buf;
};

class EncoderJPEG : public Encoder {
public:
    EncoderJPEG(StreamInfo *sInfo);
    ~EncoderJPEG();
    METHODDEF(void) my_error_exit(j_common_ptr cinfo);
    int put_frame(const Frame *frame);
    const Frame* encode_frame();

private:
    int mFmt;
    int mWidth, mHeight;

    PictureFrame mFrame;
    uint8_t *mOutBuf;
    uint64_t mOutSize;
    int mQuality; //Compression quality (0..100; 5-95 is most useful range, default is 75)

    struct jpeg_compress_struct cinfo;
    struct my_error_mgr jerr;
    // FILE* fp;
};
} // namespace roller_eye
