#ifndef __ROLLER_EYE_MOTION_DETECT__H__
#define __ROLLER_EYE_MOTION_DETECT__H__
#include<mutex>
#include"ros/ros.h"
#include"roller_eye/detect.h"
#include"roller_eye/cv_img_stream.h"
#include"roller_eye/motion_set_zone.h"
#include"roller_eye/motion_detect_enable.h"
#include <opencv2/opencv.hpp>
#include <thread>
#include"std_msgs/Int32.h"

using namespace std;
namespace roller_eye{
    class MotionDetect{
    public:
        MotionDetect();
        ~MotionDetect();
        void start();
        void stop();
        void pause() {mPause=true;}
        void resume() {mPause =false;}
        int setData(roller_eye::detect& data);
    private:
        void setROIRegion(vector<roller_eye::contour>& region);
        void setDefaultROIRegion();
        int getROIImg(cv::Mat& img);
        bool getMotionRegion(cv::Mat& mat,cv::Rect& region,double& radio);
        bool setZone(motion_set_zoneRequest& req,motion_set_zoneResponse& res);

        ros::NodeHandle mLocal;
        ros::ServiceServer mSetZone;
        CVGreyImageStream mGreyStream;
        cv::Mat mROI;
        vector<roller_eye::contour> mRegion;
        mutex mROIMutex;
        uint32_t mSeq;
        cv::Mat mPreImg;
        cv::Mat mDilateStruct;
        cv::Mat  mErodeStruct;
        int mWidth;
        int mHeigt;
        float MIN_AREA;
        float MAX_AREA;

        bool mIsRunning;
        bool mPause;
    };

    class MotionDetectMgr{
        public:
            MotionDetectMgr(shared_ptr<ros::NodeHandle>);
            ~MotionDetectMgr();
        private:
            void Loop();
            void  EnbaleMotionCallback(const std_msgs::Int32::ConstPtr& enable);
            bool enableDetect(motion_detect_enableRequest & req,motion_detect_enableResponse& res);

            bool mEnableDetect = false;
            ros::ServiceServer mEnableDetectSrv;
            shared_ptr<ros::NodeHandle> mHandle;
            std::thread mThread;
            MotionDetect mMtnDtct;
            ros::Publisher mPub;
            ros::Subscriber mSub;
    };
}
#endif