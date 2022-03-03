#ifndef __ROLLER_EYE_ALG_BACKING_UP__H__
#define __ROLLER_EYE_ALG_BACKING_UP__H__
#include<thread>
#include<vector>
#include<opencv2/opencv.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include"roller_eye/camera_handle.hpp"
#include"roller_eye/algo_utils.h"
#include"roller_eye/status_publisher.h"
#include"roller_eye/cv_img_stream.h"
#include"std_msgs/Int32.h"

#include"roller_eye/track_trace.h"
#include "roller_eye/nav_cancel.h"

namespace roller_eye
{
class BackingUp{
public:
    BackingUp(ros::NodeHandle& handle);
    ~BackingUp();
    void start();
    void stop();
    int setData(std_msgs::Int32& s);
private:
    void backupLoop();
    void updateStatus(int status);
    void pubStatus(int status);

    bool getCameraPose(float z,Eigen::Vector3d& pos,cv::Rect2f& roi, float &xdist, float &zdist,float &angle);
    bool getHomeDistanceAndAngle(const AlgoOBjPos &objPos,float &x,float&z,float &angle);
    bool getHomeDistanceAndAngleByObj(AlgoOBjPos &pos,float &angle, float&z);

    int waitObj(const string& name,int timeout,AlgoOBjPos &pos, int cmd);


    int detect(int times);
    int detectOnce();
    void doDetect();

    void doAlign();
    void doBackup();

    int moveByObj(float angle,  float z);
    int moveByCorner(float x,  float z, float angle);

    bool canceled();
    void  cancelCallback(const std_msgs::Int32::ConstPtr& msg);
    void onBatteryStatus(const statusConstPtr &s);
    bool cancel(nav_cancelRequest& req,nav_cancelResponse& res);
    bool adjustLight(int cmd);
    bool isNightMode();

    int roll(float angle,float w,int timeout=DURATION_10_S,float error=DGREE_3_RAD);
    int move(float x,float y,float speed,int timeout=DURATION_10_S,float error=0.01);

    void initEstimate();
    void deinitEstimate();
    void startAvoidObstacle();
    void avoidObstacle();
    void odomRelativeCB(const nav_msgs::Odometry::ConstPtr& msg);
    void tofDataCB(const sensor_msgs::RangeConstPtr &r);
    void doTrace(double stamp);
    void startRoll(double angle);
    bool startTrace(bool reset);
    void stopTrace();

    void reConnectToChargePile();

    bool canBackup(float x, float z, float angle);
    void timerCallback(const ros::TimerEvent& evt);
    
    StatusPublisher mPub;
    AlgoUtils mAlgo;
    AlgoOBjPos mObjPos;
    int mStatus;
    int mBackupCounter;
    std::thread mThread;
    float mBackupRoll;
    CVGreyImageStream mCVStream;
    cv::Rect2f mROI;
    bool mROIValid;
    float mAngle;

    ros::Publisher mStatusPub;     
    ros::Subscriber mCancelSub;           
    ros::Subscriber mBatteryStatus;
    ros::NodeHandle mGlobal;
    ros::Subscriber mOdomRelative;
    ros::Subscriber mTof;
    ros::Publisher mCmdVel;
    ros::ServiceServer mCancel;
    ros::ServiceClient mAdjustClient;
    ros:: ServiceClient mNightGetClient;

    ros::Subscriber mTestSub;

    ros::Timer mTimer;
    
    TrackList mCurrent;
    TrackTrace mTrace;
    Eigen::Quaterniond mCurPose;
    Eigen::Vector3d  mCurPostion;
    string mPatrolingPathName;
    bool mIsTracing;                             
    bool mDoBacking;
    double mLastTraceStamp;
   atomic<bool> mAdjusting;

    atomic<int> mDetectObstacleCnt;
    DistanceCloud mDistCloud;
    bool mClipData;
    bool mCanceled;

    bool mReconnecting;
    int mLastVoltage;
    int mCurrentVoltage;
    int mChargingCounter;
};

void differenceVector(vector<float> &data);

void lineFit(cv::Point2f* p,int n,int stride,double &k,double &b,bool tran=false);

void cvMatToEigenRotation(cv::Mat& mat,Eigen::Matrix3d& rotation);

void poseToMotion(Eigen::Vector3d& pos,Eigen::Matrix3d& rotation,float& s,float& wBegin,float& wEnd);

bool calcCameraPoseEx(float z,cv::Mat &grey,Eigen::Vector3d& pos,cv::Rect& roi, float &xdist, float &zdist,float &angle);


void calcBasePose(Eigen::Vector3d& pos,Eigen::Matrix3d& rot);

void onBatteryStatus(const statusConstPtr &s);
} // namespace roller_eye
#endif