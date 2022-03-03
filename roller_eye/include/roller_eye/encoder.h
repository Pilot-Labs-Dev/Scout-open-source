#ifndef __ROLLER_EYE_ENCODER_H__
#define __ROLLER_EYE_ENCODER_H__
#include <cstdint>
#include<memory>

using namespace std;
namespace roller_eye
{
    class Frame{
    public:
        Frame();
        Frame(const Frame& frame);
        Frame(const Frame&&)=delete;
        Frame& operator=(const Frame& frame);
        Frame& operator=(const Frame&& frame)=delete;
        virtual ~Frame();
        void setData(uint8_t* data,int size,uint64_t timestamp=0,bool dump=false);
        void setTimestamp(uint64_t timestamp);
        const uint8_t* getData()const{return mData;};
        int getSize()const{return mSize;};
        uint64_t getTimestamp()const{return mTimestamp;};

    private:
        void dumpFrame(const uint8_t *data,int size,uint64_t timestamp);
        void dumpFrame(const Frame &frame);
        void clearData();

        int mSize;
        uint8_t* mData;
        uint64_t mTimestamp;
        bool mDump;
    };
    class VideoFrame:public Frame{
    public:
        int width;
        int height;
        int rowLen;
        int fmt;
        int isKeyframe;
    };
    class PictureFrame:public Frame{
    public:
        int width;
        int height;
        int rowLen;
        int fmt;
    };
    class AudioFrame:public Frame{
    public:
        int sampleRate;
        int bitwidth;
        int channels;
    };
    class Encoder{
    public:
       Encoder();
       virtual ~Encoder();

       virtual  int put_frame(const Frame *frame)=0;

       virtual const Frame* encode_frame()=0;
    };
} // namespace roller_eye

#endif