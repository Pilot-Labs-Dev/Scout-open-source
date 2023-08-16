#ifndef ROLLER_ALGO_UTILS__H
#define ROLLER_ALGO_UTILS__H

#include<string>
#include<mutex>
#include<vector>
#include<atomic>
#include"ros/ros.h"
#include <ros/callback_queue.h>
#include"roller_eye/detect.h"
#include"sensor_msgs/Imu.h"
#include"sensor_msgs/Range.h"
#include <sensor_msgs/MagneticField.h>
#include<opencv2/opencv.hpp>
#include"roller_eye/status.h"
#include<nav_msgs/Odometry.h>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Geometry>
#include "plt_config.h"

using namespace std;

#define DGREE_1_RAD                 (0.052333/3.0) //3 dgree
#define DGREE_2_RAD                 (0.052333*2.0/3.0) //3 dgree
#define DGREE_3_RAD                 (0.052333) //3 dgree
#define DURATION_10_S             (10000)
#define MAX_VALID_TOF_DIST       2
#define ALGO_TOF_AVG_COUNT                 4

typedef std::vector<float> DistanceCloud;
typedef vector<pair<double, double>>  DistanceMap;

typedef enum {
    GET_YAW,
    GET_PITCH
} ImuCallbackType;

namespace roller_eye
{
    struct AlgoOBjPos{
        int width;
        int height;
        int left;
        int right;
        int top;
        int bottom;
        uint64 stamp;
        uint64 detectStamp;   //start detect stamp, detectStamp may be less stamp
    };
    class AlgoUtils{
    public:
        AlgoUtils();
        ~AlgoUtils();
        void waitObjBegin();
        void waitObjEnd();
        int waitObj(const string& name,int timeout,AlgoOBjPos &pos);

        int waitObj2(const string& name,int timeout,AlgoOBjPos &pos);

        void quitAlgo();
        void resetQuitFlag(){ mQuitFlag=false;}
        int roll(float angle,float w,int timeout=DURATION_10_S,float error=DGREE_3_RAD);
        int rollAndFindPile(float angle,float w,int timeout=DURATION_10_S,float error=DGREE_3_RAD);

        int rollEx(float angle,float w,int timeout=DURATION_10_S,float error=DGREE_3_RAD);

        int rollEx(float angle,float w, float minW, int timeout=DURATION_10_S,float error=DGREE_3_RAD);
        int move(float x,float y,float speed,int timeout=DURATION_10_S,float error=0.01);
        int moveEx(float x,float y,float speed, int timeout=DURATION_10_S,float error=0.01);
        int action(float speedX,float speedY,float speedW,int time);
        // int connectToChargePile(int tryCnt);
        int connectToChargePile(int tryCnt , int round);
        int connectToChargePileTrack(int tryCnt);

        int reConnectToChargePile(int timeout);
        int genDistance(DistanceCloud& dist);
        int getDistance(float angle,shared_ptr<DistanceMap> distMap);
        float getMaxSpace(DistanceCloud& dist,float distThreshold,float& arc);
        void magneticCalibrate(int timeout=4*DURATION_10_S);
    private:
        int rollAndGenDistance(float angle,float w,int timeout,float error,bool genDist,DistanceCloud& dist);
        int rollAndGenDistanceEx(float angle,float w,int timeout,float error,bool genDist,DistanceCloud& dist);
        /// @brief
        /// @param angle
        /// @param w
        /// @param minW
        /// @param timeout
        /// @param error
        /// @param genDist
        /// @param dist
        /// @param findPile
        /// @return 0       : Roll to angle successful
        ///         (-1)    : Roll failed
        ///         (1)    : Stop because the robot is charged
        int rollAndGenDistanceEx(float angle,float w,float minW, int timeout,float error,bool genDist,DistanceCloud& dist, bool findPile = false);
        /// @brief
        /// @param angle
        /// @param w
        /// @param timeout
        /// @param error
        /// @param genDist
        /// @param distMap
        /// @return 0       : Rolled and stop when reach angle
        ///         (-1)    : Rolled failed
        ///         1       : Rolled and stoped when finded a free space.
        int rollAndGenDistanceRm(float angle,float w,int timeout,float error,bool genDist,shared_ptr<DistanceMap> distMap);

        void onObjdetect(const detectConstPtr& obj);
        void onIMUData(const sensor_msgs::ImuConstPtr& imu);
        void onIMUData2(const sensor_msgs::ImuConstPtr& imu);
        void onBatteryStatus(const statusConstPtr &s);
        void onTofData(const sensor_msgs::RangeConstPtr &r);
        void onMagData(const sensor_msgs::MagneticFieldConstPtr &mag);
        void odomRelativeCB(const nav_msgs::Odometry::ConstPtr& msg);
        void moveOnce(float vx,float vy,float wz,int duration,int stop);
        void moveOnce2(float vx,float vy,float wz,int duration,int stop);
        void moveOnceForce(float vx,float vy,float wz,int duration,int stop);
        void moveOnceEx(float vx,float vy,float wz,  float dist, int duration,int stop);
        void shutdownObjDetect();
        void stopDetectPile();
        bool isCharging();
        void imuQueueThread();
        bool shouldMoveOutWhenClimb(float vy, float wz, int& cnt, float& last_wz, bool& just_climbed);
        int checkCharging();
        bool tryToContactChargePile(float vy);

        ros::NodeHandle mHandle;
        ros::Subscriber mObjDetect;
        ros::Subscriber mIMU;
        ros::Subscriber mTof;
        ros::Publisher mMotorVel;
        ros::Publisher mMotorVelForce;
        ros::Subscriber mBatteryStatus;
        ros::ServiceClient mStopDetectClient;
        ros::CallbackQueue imuCallbackQueue;
        ros::SubscribeOptions imuSubOps;
        std::thread imuRosQueueThread;

        Eigen::Quaterniond mCurPose;
        Eigen::Vector3d  mCurPostion;

        mutex mObjMutex;
        string mObj;
        AlgoOBjPos mPos;
        bool mDetected;
        bool mHasDetected;
        bool mQuitFlag;
        double mRollAngular;
        float mPitchAngular;
        uint64 mStartWaitObjStamp;
        mutex mAngleMutex;
        sensor_msgs::Imu mPreIMU;
        sensor_msgs::Imu mPrePitchIMU;
        bool mIsFirstIMU;
        int mIMUCalibraCnt;
        float mZCalibra;
        float mDistance;
		//rmwei
	    int mTofAvgCnt;
	    float mAvgTof;
	    bool mIsFirstTof = true;
        int mTofDataCnt;

        float mIMUCalibraZOffset;

        int mCharging;

        float mAccZ;
        float mAccZCalibra;
        float mIMUCalibraAccZOffset;

        bool mIsPitchFirstIMU = true;

        ros::Subscriber mMagCalibraSub;
        ros::Subscriber mOdomRelative;
        std::vector<cv::Point2f> mMagDatas;
        atomic<int> mMoveTime;
        atomic<bool> mDataReady;

        float mMinRollSpeed;
        float mMoveForwardRatio = 1.0;
        float mMoveBackwardRatio = 1.0;

        bool isClimbing = false;
        int climbing_cnt = 0;
        float mGyroxClimbingThres;
        int mMaxClimbingCnt;
        DeviceDefaultConfig mCfg;

        ros::Time t_last_imu;
        int cnt;
        int imuCallbackType_{GET_YAW};
        float mAvgWz = 0.0;             ///< average wz at 180-degree-rotation to connect to the charger.
    };
    struct MagCalibraParam{
        double x;//x offset
        double y;//y offset
        double w;//x width
        double h;//y height
        double c;//cos(angle)
        double s;//sin(angle)
    };
    void loadMagCalibraParam(MagCalibraParam& param);
    void magCalibrate(double& x,double&y,double&z,MagCalibraParam& param);
} // namespace roller_eye


#endif
