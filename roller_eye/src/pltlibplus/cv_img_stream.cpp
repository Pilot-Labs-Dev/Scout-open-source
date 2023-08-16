 #include"cv_img_stream.h"
#include"roller_eye/camera_handle.hpp"
#include"roller_eye/graphic_utils.h"

 namespace roller_eye{
    CVGreyImageStream::CVGreyImageStream(int width,int height):
    mWidth(width),
    mHeight(height),
    mStream(NULL),
    mNeedResume(false)
    {
        CameraHandle::getInstance()->registParamListener(std::bind(&CVGreyImageStream::onParamChanged, this, placeholders::_1, placeholders::_2));
    }
    CVGreyImageStream::~CVGreyImageStream()
    {
        stopStream();
    }

    void CVGreyImageStream::onParamChanged(CaputreParam *param, bool resume)
    {
        // ROS_INFO("CVGreyImageStream::onParamChanged >>>%d, %d, %d, 0x%x",
                                    // param->sInfo.fInfo.width, param->sInfo.fInfo.height, param->sInfo.fpsDen, param->sInfo.fInfo.fmt);
        if ((!resume && !mStream) ||
            (resume && !mNeedResume)) {
            mNeedResume = false;
            return;
        }
        if (!resume){
            mNeedResume = true;
            stopStream();
            // ROS_INFO("CVGreyImageStream::onParamChanged >>> stop END");
        }else {
            mNeedResume = false;
            // ROS_INFO("CVGreyImageStream::onParamChanged >>> END");
        }
    }

    int CVGreyImageStream::getCVImg(cv::Mat &img,bool autoRetry)
    {
        if (mNeedResume){
            return -1;
        }
        lock_guard<recursive_mutex> lock(mReMutex);
        if(mStream==NULL){
            if((mStream=CameraHandle::getInstance()->createVideoStream(1))==NULL){
                return -1;
            }
        }
        FrameBuff *frame=video_stream_get_frame(mStream);
        if(frame==NULL){
            if(!autoRetry){
                return -1;
            }
            stopStream();
            return getCVImg(img,false);
        }
        int w=mWidth;
        int h=mHeight;
        GraphicUtils::getInstance()->decodeYDataAndTryScale((unsigned char*)frame->addr,frame->fInfo.width,frame->fInfo.height,frame->fInfo.fmt,mGreyData,w,h);
        video_stream_return_frame(mStream,frame);
        img=cv::Mat(h,w,CV_8UC1,mGreyData.data());
        return 0;
    }
    void CVGreyImageStream::stopStream()
    {
        lock_guard<recursive_mutex> lock(mReMutex);
        if(mStream!=NULL){
            video_stream_destory(mStream);
            mStream=NULL;
        }
    }
    void getROICVImg(cv::Mat& src,cv::Mat& dst,cv::Rect2f &roi)
    {
         cv::Rect rect(round(roi.x*src.cols),round(roi.y*src.rows),round(roi.width*src.cols),round(roi.height*src.rows));

        rect.x=std::max(0,rect.x);
        rect.x=std::min(rect.x,src.cols-1);
        rect.width=std::min(rect.width,src.cols-rect.x);
        rect.y=std::max(0,rect.y);
        rect.y=std::min(rect.y,src.rows-1);
        rect.height=std::min(rect.height,src.rows-rect.y);
        dst=src(rect);
    }
 }