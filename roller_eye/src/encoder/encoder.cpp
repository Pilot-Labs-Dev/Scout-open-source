#include"encoder.h"
#include"plog_plus.h"
namespace roller_eye
{
    Encoder::Encoder()
    {   

    }
    Encoder::~Encoder()
    {

    }  
    Frame::Frame():
    mSize(0),
    mData(nullptr),
    mTimestamp(0),
    mDump(false)
    {
    } 
    Frame::Frame(const Frame& frame):
    mSize(0),
    mData(nullptr),
    mTimestamp(0),
    mDump(false)
    {
        dumpFrame(frame);
    }
     Frame& Frame::operator=(const Frame& frame)
     {
         dumpFrame(frame);
         return *this;
     }
     Frame::~Frame()
     {
         clearData();
     }
    void Frame::setData(uint8_t* data,int size,uint64_t timestamp,bool dump)
    {
        clearData();
        if(dump){
            dumpFrame(data,size,timestamp);
        }else{
            if(data!=nullptr&&size>0){
                mData=data;
                mSize=size;
                mTimestamp=timestamp;
            }
        }
    }
    void Frame::setTimestamp(uint64_t timestamp)
    {
        mTimestamp=timestamp;
    }
     void Frame::dumpFrame(const uint8_t *data,int size,uint64_t timestamp)
     {
        PLOG_WARN("Frame","Dump a Frame,size=%d\n",size);

        clearData();
        
        if(size>0&&data!=nullptr){
            mTimestamp=timestamp;
            mSize=size;
            mData=new uint8_t[size];
            copy(&data[0],&data[size],&mData[0]);
            mDump=true;
        }
     }
     void Frame::dumpFrame(const Frame &frame)
     {
         dumpFrame(frame.mData,frame.mSize,frame.mTimestamp);
     }
    void Frame::clearData()
    {
        if(mDump){
            delete mData;
        }
        mDump=false;
        mData=nullptr;
        mSize=0;
        mTimestamp=0;
    }
} // namespace roller_eye

