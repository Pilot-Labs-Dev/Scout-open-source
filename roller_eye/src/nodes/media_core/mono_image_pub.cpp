#include<unistd.h>
#include"mono_image_pub.h"
#include"camera_handle.hpp"
#include"graphic_utils.h"
#include "sensor_msgs/image_encodings.h"
#include"roller_eye/system_define.h"

#define EPOCH_FRESH_CNT                     1000
#define EPOCH_SHIFT_MAX_ERROR      200       //unit: us

namespace roller_eye{
    MonoImgPub::MonoImgPub():mVideoStream(NULL),mNeedResume(false)
    {
        mFilter=0;
        mSkipCnt=1;
        mFirstFrame=true;
        mLastEpchoShift=-1;
        mEpchoFreshCnt=0;
        CameraHandle::getInstance()->registParamListener(std::bind(&MonoImgPub::onParamChanged, this,
                                                  placeholders::_1, placeholders::_2));
    }
    MonoImgPub::~MonoImgPub()
    {
        freeVideoStream();
    }
    void MonoImgPub::start()
    {
        getVideoStream();
        mFirstFrame=true;
        mEpchoFreshCnt=0;
        mLastEpchoShift=-1;
    }
    void MonoImgPub::stop()
    {
       freeVideoStream();
    }
    void MonoImgPub::onParamChanged(CaputreParam *param, bool resume)
    {
    if ((!resume && !mVideoStream)||
         (resume && !mNeedResume)) {
        mNeedResume = false;
        return;
    }
    if (!resume){
        mNeedResume = true;
        stop();
    }else{
        mNeedResume = false;
        start();
    }
}


    void MonoImgPub::freeVideoStream()
    {
         if(mVideoStream!=NULL){
            video_stream_destory(mVideoStream);
            mVideoStream=NULL;
        }
    }
    void MonoImgPub::getVideoStream()
    {
        freeVideoStream();
        mVideoStream=CameraHandle::getInstance()->createVideoStream();
        auto par=CameraHandle::getInstance()->getCaptureParam();
        mFilter=0;
        mSkipCnt=par.sInfo.fpsDen/(par.sInfo.fpsNum*MONO_PUB_FPS);
        if(mSkipCnt==0){
            mSkipCnt=1;
        }
        ROS_DEBUG("skip cnt:%d",mSkipCnt);
    }
    int64_t MonoImgPub::getEpochMonoShift()
    {
        struct timeval epoch;
        struct timespec  mono0;
        struct timespec  mono1;

        if(mEpchoFreshCnt==0||mLastEpchoShift<0){
            int64_t deta;
            while(true){
                clock_gettime(CLOCK_MONOTONIC, &mono0);
                gettimeofday(&epoch, NULL);
                clock_gettime(CLOCK_MONOTONIC, &mono1);
                deta=(int64_t)(mono1.tv_sec-mono0.tv_sec)*1000+(mono1.tv_nsec-mono0.tv_nsec)/1000;
                if(deta<EPOCH_SHIFT_MAX_ERROR){
                    break;
                }
                ROS_DEBUG("mono calibra:task schedule out,time:%ld us,retry!!!",deta);
            }
            mLastEpchoShift=(int64_t)epoch.tv_sec*1000000+epoch.tv_usec-(int64_t)(mono0.tv_sec+mono1.tv_sec)*500000-(mono0.tv_nsec+mono1.tv_nsec)/2000;
            ROS_DEBUG("mLastEpchoShift=%ld",mLastEpchoShift);
        }
        if(++mEpchoFreshCnt>=EPOCH_FRESH_CNT){
            mEpchoFreshCnt=0;
        }
        return mLastEpchoShift;
    }
    int MonoImgPub::setData(sensor_msgs::Image& s)
    {
        int ret=-1,w,h;
        bool skip=(mFilter++%mSkipCnt!=0);

        if(mVideoStream!=NULL){
            FrameBuff *frame=video_stream_get_frame(mVideoStream);
            if(frame!=NULL){
                if(!skip){
                    if(mFirstFrame){
                        mFirstFrame=false;
                        mLastTimeVal=frame->stamp;
                    }
#ifdef SLAM_PRINTF_TIME_DIFF
                    else
                    {
                        int diff=plt_timeval_diff_ms(&frame->stamp,&mLastTimeVal);
                        if(abs(diff-40)>=10){
                            ROS_WARN("stamp diff:%d",diff);
                        }
                        mLastTimeVal=frame->stamp;
                    }
#endif
#ifdef  SLAM_USE_KERNEL_STAMP
                    s.header.stamp.fromNSec(plt_timeval_to_ns(&frame->stamp)+getEpochMonoShift()*1000);
#else
                    s.header.stamp=ros::Time::now();
#endif

                    w=MONO_PUB_SUGEST_W;
                    h=MONO_PUB_SUGEST_H;
                    GraphicUtils::getInstance()->decodeYDataAndTryScale((unsigned char*)frame->addr,frame->fInfo.width,frame->fInfo.height,frame->fInfo.fmt,s.data,w,h);

                    s.is_bigendian=0;
                    s.height=h;
                    s.width=w;
                    s.step=s.width;
                    s.encoding=sensor_msgs::image_encodings::MONO8;
                    ret=0;
               }
                video_stream_return_frame(mVideoStream,frame);
            }else{//maybe reset resolution
                getVideoStream();
            }
        }
        if(ret<0&&!skip){
            usleep(30*1000);
        }
        return ret;
    }
}