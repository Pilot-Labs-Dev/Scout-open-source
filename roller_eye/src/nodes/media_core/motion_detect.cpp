#include<unistd.h>
#include"motion_detect.h"
#include"roller_eye/plog_plus.h"
#include"roller_eye/util_class.h"
#include"roller_eye/param_utils.h"
#include"roller_eye/plt_config.h"
using namespace cv;

#define MOTION_DETECT_W         640
#define MOTION_DETECT_H          480

#ifdef USE_DETECT_METHOD_1
#define MOTION_MORTH_SIZE       30
#else
#define MOTION_MORTH_SIZE       1
#endif

#define MOTION_THROLD                  30
#define MOTION_GAUSS_SIZE          5


#define MOTION_DETECT_TAG "MotionDetect"
namespace roller_eye{
        MotionDetect::MotionDetect():
        mLocal("~"),
        mGreyStream(MOTION_DETECT_W,MOTION_DETECT_H),
        mSeq(0),
        mWidth(MOTION_DETECT_W),
        mHeigt(MOTION_DETECT_H),
        mIsRunning(false),
        mPause(false)
        {
            DeviceDefaultConfig cfg;

            MIN_AREA=cfg.getMotionMinArea();
            MAX_AREA=cfg.getMotionMaxArea();

            mSetZone=mLocal.advertiseService("motion_set_zone",&MotionDetect::setZone,this);
            int size=2*MOTION_MORTH_SIZE+1;
            mDilateStruct=getStructuringElement(cv::MORPH_RECT, cv::Size(size, size));
            mErodeStruct=getStructuringElement(cv::MORPH_RECT, cv::Size(size, size));
            setDefaultROIRegion();
        }
        MotionDetect::~MotionDetect()
        {

        }
        void MotionDetect::start()
        {
            mIsRunning = true;
        }
        void MotionDetect::stop()
        {
            mIsRunning = false;
            mGreyStream.stopStream();
            mPreImg.release();
        }
        int MotionDetect::setData(roller_eye::detect& data)
        {
            if (!mIsRunning || mPause){
                return -1;
            }
            TimeCost c;
            memset(&data,0,sizeof(data));
            cv::Mat roiImg;
            if(getROIImg(roiImg)<0){
                usleep(30000);
                return -1;
            }
            cv::Rect region;
            double radio;

            if(!getMotionRegion(roiImg,region,radio)){
                return -1;
            }

            if(radio<MIN_AREA || radio>MAX_AREA){
                PLOG_DEBUG(MOTION_DETECT_TAG,"ignore motion area = %f",radio);
                return -1;
            }
            if (mPause){
                return -1;
            }

            data.score=1.0;
            data.index=0;
            data.seq=mSeq++;
            data.name="motion";
            data.width=mWidth;
            data.height=mHeigt;
            data.left=region.x;
            data.top=region.y;
            data.right=data.left+region.width;
            data.bottom=data.top+region.height;
            return 0;
        }
        void MotionDetect::setROIRegion(vector<roller_eye::contour>& region)
        {
            vector<vector<cv::Point >> contours;
            int value;
#ifndef MOTION_DETECT_DEBUG
            cv::Mat tmp=cv::Mat(MOTION_DETECT_H,MOTION_DETECT_W,CV_8UC1,cv::Scalar(255));
#else
            cv::Mat tmp=cv::Mat(mHeigt,mWidth,CV_8UC1,cv::Scalar(255));   //motion test modify for test by ltl 2021-03-12
#endif
            for(auto& r:region){
                value=r.inside?255:0;
                contours.clear();
                vector<cv::Point> contour;
                cv::Point point;
                for(auto& p:r.points){
                    point.x=std::max(0,(int)round(p.x*mWidth));
                    point.x=std::min(point.x,mWidth-1);
                    //point.x=std::min(point.x,MOTION_DETECT_W-1);   //add by ltl 2021-03-15
                    point.y=std::max(0,(int)round(p.y*mHeigt));
                    point.y=std::min(point.y,mHeigt-1);
                   // point.y=std::min(point.y,MOTION_DETECT_H-1);   //add by ltl 2021-03-15
                    PLOG_INFO(MOTION_DETECT_TAG,"ROI contour:x=%d,y=%d",point.x,point.y);
                    contour.push_back(point);
                }
                contours.push_back(std::move(contour));
#ifndef MOTION_DETECT_DEBUG  //motion test modify for test by ltl 2021-03-12
                cv::Mat mask=cv::Mat(MOTION_DETECT_H,MOTION_DETECT_W,CV_8UC1,cv::Scalar(255-value));
#else
                cv::Mat mask=cv::Mat(mHeigt,mWidth,CV_8UC1,cv::Scalar(255-value));
#endif
                cv::polylines(mask, contours, true, cv::Scalar(value));
                cv::fillPoly(mask, contours, cv::Scalar(value));
                cv::Mat img;

                PLOG_INFO(MOTION_DETECT_TAG,"ROI tmp size %d %d , mask size %d %d",tmp.size().width,
                                                                                tmp.size().height,  mask.size().width, mask.size().height);

                tmp.copyTo(img,mask);
                tmp=img;
            }
            lock_guard<mutex> lock(mROIMutex);
            mROI=tmp;
            mRegion=region;
        }
        void MotionDetect::setDefaultROIRegion()
        {
            PLOG_INFO(MOTION_DETECT_TAG,"%s %d",__FILE__, __LINE__);
            MonitorParam param;
            load_monitor_param(param);
            setROIRegion(param.zone.contours);
        }
        int MotionDetect::getROIImg(cv::Mat& img)
        {
            cv::Mat grey;
            if(mGreyStream.getCVImg(grey)<0){
                return -1;
            }
            mWidth=grey.cols;
            mHeigt=grey.rows;
            if(mROI.cols!=mWidth || mROI.rows!=mHeigt){
                PLOG_INFO(MOTION_DETECT_TAG,"%s %d",__FILE__, __LINE__);
                setROIRegion(mRegion);
            }
            lock_guard<mutex> lock(mROIMutex);
            grey.copyTo(img,mROI);
            return 0;
        }
#ifdef USE_DETECT_METHOD_1
        bool MotionDetect::getMotionRegion(cv::Mat& mat,cv::Rect& region,double& radio)
        {
            cv::Mat gauss;
            GaussianBlur(mat, gauss, cv::Size(MOTION_GAUSS_SIZE, MOTION_GAUSS_SIZE),2,2);
            if(mPreImg.empty()){
                PLOG_DEBUG(MOTION_DETECT_TAG,"first image");
                mPreImg=gauss;
                return false;
            }
            cv::Mat diff;
            absdiff(gauss,mPreImg,diff);
            cv::Mat bin;
            threshold(diff,bin,MOTION_THROLD, 255,CV_THRESH_BINARY);
            cv::Mat dila;
            dilate(bin, dila, mDilateStruct);
            cv::Mat ero;
            erode(dila,ero,mErodeStruct);
            vector<vector<cv::Point> > contours;
            findContours(ero, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
            if(!contours.empty()){
                region=boundingRect(contours[0]);
                radio=(double)region.width*region.height/(mat.cols*mat.rows);
            }
            mPreImg=gauss;
            return !contours.empty();
        }
#else
    bool MotionDetect::getMotionRegion(cv::Mat& mat,cv::Rect& region,double& radio)
    {
    	bool hasRegion=false;
    	int region_count = 0;
    	static int startup =0;
		Mat Mask;
		std::vector<std::vector<Point>>contours;
		Ptr<BackgroundSubtractor> Background = createBackgroundSubtractorKNN(500,400.0,false);

		Background->apply(mat, Mask);
		if(10 > startup++ )return hasRegion;
		else startup = 10;

		threshold(Mask, Mask, 100, 255, THRESH_BINARY);
		Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
		morphologyEx(Mask, Mask, MORPH_OPEN, kernel, Point(-1,-1));
		findContours(Mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0,0));
		for (int i = 0; i < contours.size(); i++){
			double area = contourArea(contours[i]);
			Rect rect = boundingRect(contours[i]);
			if (area < 1000 || rect.width < 50 || rect.height < 50) continue;
			region = rect;
			region_count++;
			hasRegion=true;
			radio = area/(mat.cols*mat.rows);
		}
		return hasRegion;
    }
#endif
    bool MotionDetect::setZone(motion_set_zoneRequest& req,motion_set_zoneResponse& res)
    {
        PLOG_INFO(MOTION_DETECT_TAG,"%s %d",__FILE__, __LINE__);
        setROIRegion(req.contours);
        return true;
    }

MotionDetectMgr::MotionDetectMgr(shared_ptr<ros::NodeHandle> n):
    mEnableDetect(false),
    mHandle(n),
    mThread([this](){
       Loop();
        })
{
    mSub = mHandle->subscribe("EnableMotion", 10,
                    &MotionDetectMgr::EnbaleMotionCallback, this);
    mPub = mHandle->advertise<roller_eye::detect>("motion",100);
    mEnableDetectSrv=mHandle->advertiseService("motion_detect_enable",
                                        &MotionDetectMgr::enableDetect,this);
}
MotionDetectMgr::~MotionDetectMgr()
{
    mThread.join();
}

bool MotionDetectMgr::enableDetect(motion_detect_enableRequest & req,motion_detect_enableResponse& res)
{
    PLOG_INFO(MOTION_DETECT_TAG,"enableDetect start ");
    if (0 == req.enable){
        mEnableDetect = false;
        mMtnDtct.stop();
    }else if (1==req.enable){
        mEnableDetect = true;
        mMtnDtct.start();
    }
    PLOG_INFO(PARAM_UTIL_TAG,"enableDetect end");
    return true;
}
void MotionDetectMgr::Loop()
{
    while(true){
        if (mEnableDetect){
                    roller_eye::detect data;
                    if(mMtnDtct.setData(data)==0){
                        mPub.publish(data);
                    }else{
                        usleep(10000);
                    }
        }else{
                usleep(10*1000);
        }
    }
}

void   MotionDetectMgr::EnbaleMotionCallback(const std_msgs::Int32::ConstPtr& enable)
{
    if (0 == enable->data){
        mEnableDetect = false;
        mMtnDtct.stop();
    }else if (1==enable->data){
        mEnableDetect = true;
        mMtnDtct.start();
    }
}

}