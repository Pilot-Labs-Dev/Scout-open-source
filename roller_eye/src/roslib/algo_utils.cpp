#include<unistd.h>
#include<cmath>
#include <thread>
#include"roller_eye/algo_utils.h"
#include"geometry_msgs/Twist.h"
#include"roller_eye/system_define.h"
#include"roller_eye/util_class.h"
#include "roller_eye/stop_detect.h"
#include "roller_eye/system_define.h"
#include "roller_eye/param_utils.h"
#include <tf2/LinearMath/Vector3.h>

#define G_NORMAL     9.7833
#define WAITING_OBJ_TIME       50

#define MIN_ROLL_SPEED              1.1f
#define FS8003_MIN_ROLL_SPEED       1.1f   ///<<< RogerL tune for FS8003 motor  1.4f
#define TRACKTYPE_MIN_ROLL_SPEED    2.0f   ///<<< Min roll speed for track type
#define IMU_WATING_TIME            100
#define ROLL_DURATION                 20
#define ROLL_IMU_CALIBRA_CNT  20
#define NEW_CALIB_IMU
#define USE_ROLL_PID

#define MAX_BACKUP_ROLL_SPEED     (mCfg.getBackupMaxRollSpeed())  ///1.33f
#define MAX_BACKUP_SHIFT_SPEED	  (mCfg.getBackupMaxShiftSpeed())  ///0.062f

#define CONNECT_CHARGE_PILE_ONCE_TIME   250

#define GEN_DISTANCE_SPEED          1.5
#define MIN_GEN_DISTANCE_SPEED          1.3
#define TRACK_GEN_DISTANCE_SPEED     2.1
#define MAX_ROLL_SPEED              5.5f

#define MAG_CALIBRA_PARAM_CNT       5
#define MAG_CALIBRA_GAIN                        (1.0e5)

#define ALGO_UTIL_TAG                         "Algo_Util"
#define USE_COMMON_IMU_CB
#define MOTION_CONFIG_PARAM_DEFAULT         "/var/roller_eye/config/motion_default"
#define OBSTACLE_MIN_DIST           0.05f
#define FREE_SPACE_DIST             0.5f
using namespace std;

namespace roller_eye
{
    AlgoUtils::AlgoUtils():
    mHandle(""),
    mQuitFlag(false),
    mAvgTof(MAX_VALID_TOF_DIST),
    mTofAvgCnt(ALGO_TOF_AVG_COUNT),
    mTofDataCnt(0),
    mDistance(MAX_VALID_TOF_DIST),
    mHasDetected(false)
    {
		PLOG_INFO(ALGO_UTIL_TAG,"Footprint(0001_...) \n");
        mMotorVel=mHandle.advertise<geometry_msgs::Twist>("cmd_vel",50);
        mMotorVelForce =mHandle.advertise<geometry_msgs::Twist>("cmd_vel_force",50);

        mStopDetectClient = mHandle.serviceClient<stop_detect>("CoreNode/stop_detect");
        mMinRollSpeed = MIN_ROLL_SPEED;

        //HuyNV-May12: set default motion_params if not exist
        PltConfig *config = PltConfig::getInstance();
        json motion_param;
        config->getMotionParam(motion_param);

        if ((int)(motion_param.size()) == 0)
        {
            PLOG_INFO(ALGO_UTIL_TAG, "motion params not exist, set default param");
            ifstream ifs(MOTION_CONFIG_PARAM_DEFAULT);
            ifs >> motion_param;
            // std::cout << "motion default params: " << motion_param << endl;;
            string hwVer;
            config->getHwVersion(hwVer);
            PLOG_DEBUG(ALGO_UTIL_TAG, "HwVer: %s", hwVer.c_str());

            if(hwVer.length() == 10 && hwVer[0] == '3'){
              if((hwVer[1] == '1') || (hwVer[1] == '2')) {
                motion_param["track"] = 0;
              }
              if (hwVer[1] == '3') {
                motion_param["track"] = 1;
              }
              std::cout << "motion param was set following hw version: " << motion_param << endl;;
              config->setMotionParam(motion_param);
            } else {
                PLOG_WARN(ALGO_UTIL_TAG, "Could not read hw version.");
            }
        }

        if (PltConfig::getInstance()->isChinaMotor()){     /// Fix mMinRollSpeed for motors
            mMinRollSpeed = FS8003_MIN_ROLL_SPEED;
			PLOG_INFO(ALGO_UTIL_TAG,"Footprint(000_China Motor) \n");
        }
        if (PltConfig::getInstance()->isTrackModel()) /// Track model ...
		{
			mMinRollSpeed = TRACKTYPE_MIN_ROLL_SPEED;   /// min roll speed for belt ...
		}
        //mTof=mHandle.subscribe("SensorNode/tof",1,&AlgoUtils::onTofData,this);

        // DeviceDefaultConfig cfg;

        mMoveForwardRatio = mCfg.getTrackTypeMoveForwardRatio();
        mMoveBackwardRatio = mCfg.getTrackTypeMoveBackwardRatio();

        imuSubOps = ros::SubscribeOptions::create<sensor_msgs::Imu>(
            "/SensorNode/imu",
            100,
            boost::bind(&AlgoUtils::onIMUData, this, _1),
            ros::VoidPtr(),
            &this->imuCallbackQueue
        );
        this->imuRosQueueThread =
          std::thread(std::bind(&AlgoUtils::imuQueueThread, this));

		PLOG_INFO(ALGO_UTIL_TAG,"AlgoUtils Constructor() \n");
    }
    AlgoUtils::~AlgoUtils()
    {
		PLOG_INFO(ALGO_UTIL_TAG,"Footprint(0002_...) \n");

    }

    void AlgoUtils::waitObjBegin()
    {
		PLOG_INFO(ALGO_UTIL_TAG,"Footprint(0003_...) \n");
        if(!mObjDetect){
            mObjDetect=mHandle.subscribe("CoreNode/chargingPile",10,&AlgoUtils::onObjdetect,this);
        }
    }
    void AlgoUtils::waitObjEnd()
    {
		PLOG_INFO(ALGO_UTIL_TAG,"Footprint(0004_...) \n");
        shutdownObjDetect();
    }

    int AlgoUtils::waitObj(const string& name,int timeout,AlgoOBjPos &pos)
    {
        bool objDetectOpen=false;

        struct timeval tm;
        gettimeofday(&tm, nullptr);
		PLOG_INFO(ALGO_UTIL_TAG,"Footprint(0005_...) \n");
        usleep(100*1000);
        // ROS_INFO("AlgoUtils::waitObj enter\n");
        if(!mObjDetect){
            mObjDetect=mHandle.subscribe("CoreNode/chargingPile",1,&AlgoUtils::onObjdetect,this);
            objDetectOpen=true;
        }
        mObjMutex.lock();
        mStartWaitObjStamp = tm.tv_sec * 1000000 + tm.tv_usec;
        mObj=name;
        mDetected=false;
        int ret = -1;
        mObjMutex.unlock();
        for(int t=0;timeout<0 || t< timeout;t+=WAITING_OBJ_TIME){
            if (mQuitFlag){
                break;
            }
            usleep(WAITING_OBJ_TIME*1000);
            if(mDetected){
                lock_guard<mutex> lock(mObjMutex);
                if(mDetected){
                    pos=mPos;
                    // ROS_INFO("waitObj get obj:%ld,obj->stamp:%ld",mStartWaitObjStamp, pos.stamp);
                    ret = 0;
                    break;
                }else{
                    continue;
                }
            }
        }

        if(objDetectOpen){
            shutdownObjDetect();
        }
        resetQuitFlag();
        if (!mDetected){
            // ROS_INFO("waitObj get obj failured!\n");
		    PLOG_INFO(ALGO_UTIL_TAG,"Footprint(0006_...) \n");
        }
        return ret;
    }

/*
    name: wait obj name
    timeout: wait detect longest time
    pos: obj position

    return: 0-detect obj and pos is valid, 1-detect obj but pos is invalid, -1 not detected obj
*/
    int AlgoUtils::waitObj2(const string& name,int timeout,AlgoOBjPos &pos)
    {
        bool objDetectOpen=false;

        // ROS_INFO("AlgoUtils::waitObj enter\n");
		PLOG_INFO(ALGO_UTIL_TAG,"Footprint(0007_...) \n");
        if(!mObjDetect){
            mObjDetect=mHandle.subscribe("CoreNode/chargingPile",1,&AlgoUtils::onObjdetect,this);
            objDetectOpen=true;
        }
        struct timeval tm;
        gettimeofday(&tm, nullptr);
        mObjMutex.lock();
        mStartWaitObjStamp = tm.tv_sec * 1000000 + tm.tv_usec;
        mObj=name;
        mDetected=false;
        mHasDetected = false;
        int ret = -1;
        mObjMutex.unlock();
        for(int t=0;timeout<0 || t< timeout;t+=WAITING_OBJ_TIME){
            if (mQuitFlag){
                break;
            }
            usleep(WAITING_OBJ_TIME*1000);
            if(mDetected){
                lock_guard<mutex> lock(mObjMutex);
                if(mDetected){
                    pos=mPos;
                    ret = 0;
                    // ROS_INFO("waitObj get obj:%ld,obj->stamp:%ld",mStartWaitObjStamp, pos.stamp);
                    break;
                }else if (mHasDetected){
                    ret = 1;
                    break;
                }else{
                    continue;
                }
            }
        }

        if(objDetectOpen){
            shutdownObjDetect();
        }
        resetQuitFlag();
        if (!mDetected){
            ROS_INFO("waitObj get obj failured!\n");
        }
        return ret;
    }

    void AlgoUtils::quitAlgo()
    {
		PLOG_INFO(ALGO_UTIL_TAG,"Footprint(0008_...) \n");
		mQuitFlag= true;
        shutdownObjDetect();
    }


    void AlgoUtils::stopDetectPile()
    {
		PLOG_INFO(ALGO_UTIL_TAG,"Footprint(0009_...) \n");
        // ROS_INFO("AlgoUtils %s %d\n",__FILE__, __LINE__);
        auto th=std::thread([this](){
            stop_detect req;
            req.request.cmd = 0;
            mStopDetectClient.call(req);
        });
        th.detach();
    }

    void AlgoUtils::shutdownObjDetect()
    {
		PLOG_INFO(ALGO_UTIL_TAG,"Footprint(000A_...) \n");
        if (mObjDetect){
            mObjDetect.shutdown();
        }

        stopDetectPile();

    }

    void AlgoUtils::moveOnce(float vx,float vy,float wz,int duration,int stop)
    {
        // PLOG_DEBUG(ALGO_UTIL_TAG, "[MoveOnce] vy: %f, wz: %f\n", vy, wz);
        const int PUB_DURATION=100;
        geometry_msgs::Twist vel;

        vel.linear.x=vx;
        vel.linear.y=vy;
        vel.angular.z=wz;

        struct timeval tv;
        struct timeval now;

        gettimeofday(&tv, NULL);
        long long starttm = tv.tv_sec*1000+tv.tv_usec/1000;
        long long curtm = tv.tv_sec*1000+tv.tv_usec/1000;

        int nCnt = 0;
        while(curtm<starttm+duration){
            mMotorVel.publish(vel);
            int uts = std::min(static_cast<int>(starttm+duration-curtm),PUB_DURATION);
            usleep(uts*1000);
            gettimeofday(&now, NULL);
            curtm = now.tv_sec*1000+now.tv_usec/1000;
        }

        if(stop>0){
            vel.linear.x=vel.linear.y=vel.angular.z=0.0;
            mMotorVel.publish(vel);
            usleep(stop*1000);
        }
    }


    void AlgoUtils::moveOnce2(float vx,float vy,float wz,int duration,int stop)
    {
        const int PUB_DURATION=100;
        geometry_msgs::Twist vel;

		PLOG_INFO(ALGO_UTIL_TAG,"moveOnce2(%f,%f,%f,%d,%d ) \n",vx,vy,wz,duration,stop);
		///PLOG_INFO(ALGO_UTIL_TAG,"Footprint(000B_...) \n");

        vel.linear.x=vx;
        vel.linear.y=vy;
        vel.angular.z=wz;

        struct timeval tv;
        struct timeval now;
        //vel.linear.x=vel.linear.y=vel.angular.z=0.0;
        vel.linear.x=vel.angular.z=0.0;
        mMotorVel.publish(vel);
        usleep(100*1000);

        gettimeofday(&tv, NULL);
        long long starttm = tv.tv_sec*1000+tv.tv_usec/1000;
        long long curtm = tv.tv_sec*1000+tv.tv_usec/1000;

        int nCnt = 0;
        while(curtm<starttm+duration){
            mMotorVel.publish(vel);
            int uts = std::min(static_cast<int>(starttm+duration-curtm),PUB_DURATION);
            usleep(uts*1000);
            gettimeofday(&now, NULL);
            curtm = now.tv_sec*1000+now.tv_usec/1000;
        }

        if(stop>0){
            vel.linear.x=vel.linear.y=vel.angular.z=0.0;
            mMotorVel.publish(vel);
            usleep(stop*1000);
        }
    }

    void AlgoUtils::moveOnceForce(float vx,float vy,float wz,int duration,int stop)
    {
        PLOG_DEBUG(ALGO_UTIL_TAG, "[MoveOnceForce] vy: %f, wz: %f, duration: %i\n", vy, wz, duration);
        const int PUB_DURATION=10;
        geometry_msgs::Twist vel;

        vel.linear.x=vx;
        vel.linear.y=vy;
        vel.angular.z=wz;

        struct timeval tv;
        struct timeval now;

        gettimeofday(&tv, NULL);
        long long starttm = tv.tv_sec*1000+tv.tv_usec/1000;
        long long curtm = tv.tv_sec*1000+tv.tv_usec/1000;

        int nCnt = 0;
        while(curtm<starttm+duration){
            mMotorVelForce.publish(vel);
            int uts = std::min(static_cast<int>(starttm+duration-curtm),PUB_DURATION);
            usleep(uts*1000);
            gettimeofday(&now, NULL);
            curtm = now.tv_sec*1000+now.tv_usec/1000;
            mAngleMutex.lock();
            float xAngle = mPitchAngular;
            mAngleMutex.unlock();
            if(abs(xAngle) > mGyroxClimbingThres && vy < 0){
                PLOG_INFO(ALGO_UTIL_TAG, "[moveone 3] ******robot is climbing... Stop...");
                break;
            }
            if (mCharging > 0) {
                PLOG_DEBUG(ALGO_UTIL_TAG, "Detected charging during moving");
                break;
            }
        }

        if(stop>0){
            vel.linear.x=vel.linear.y=vel.angular.z=0.0;
            mMotorVelForce.publish(vel);
            usleep(stop*1000);
        }
    }

    void AlgoUtils::moveOnceEx(float vx,float vy,float wz,float dist, int duration,int stop)
    {
		PLOG_INFO(ALGO_UTIL_TAG,"moveOnceEx(%f,%f,%f,%d,%d, %f ) \n",vx,vy,wz,duration,stop,dist);
		///PLOG_INFO(ALGO_UTIL_TAG,"Footprint(000C_...) \n");
        if(!mOdomRelative){
            mOdomRelative=mHandle.subscribe("MotorNode/baselink_odom_relative", 100, &AlgoUtils::odomRelativeCB,this);
        }

        mCurPose=Eigen::Quaterniond(1.0,0.0,0.0,0.0);
        mCurPostion=Eigen::Vector3d(0.0,0.0,0.0);

        const int PUB_DURATION=10;
        geometry_msgs::Twist vel;

        vel.linear.x=vx;
        vel.linear.y=vy;
        vel.angular.z=wz;

        struct timeval tv;
        struct timeval now;

        gettimeofday(&tv, NULL);
        long long starttm = tv.tv_sec*1000+tv.tv_usec/1000;
        long long curtm = tv.tv_sec*1000+tv.tv_usec/1000;

        Eigen::Vector3d pos = Eigen::Vector3d(0.0,0.0,0.0);

        float dt = 0;
        while(dt<dist && curtm<starttm+duration){
            mMotorVel.publish(vel);
            usleep(std::min(static_cast<int>(starttm+duration-curtm),PUB_DURATION)*1000);
            gettimeofday(&now, NULL);
            curtm = now.tv_sec*1000+now.tv_usec/1000;
            Eigen::Vector3d d=mCurPose.inverse()*(mCurPostion-pos);
            dt = d.norm();
        }


        if(stop>0){
            vel.linear.x=vel.linear.y=vel.angular.z=0.0;
            mMotorVel.publish(vel);
            usleep(stop*1000);
        }

       if(mOdomRelative){
            mOdomRelative.shutdown();
       }
    }

    /**
     * @description: before do roll, car will be stopped first
     * @param angle: rotated angle, unit is rad, the rotated direction is -?
     * @param w: rotated speed, unit is rad/s
     * @param timeout:
     * @param error  -?
     * @return ==0 true, ==others false
     */
    int AlgoUtils::roll(float angle,float w,int timeout,float error)
    {
        if (PltConfig::getInstance()->isTrackModel())
		{
            mMinRollSpeed = TRACKTYPE_MIN_ROLL_SPEED;
            if(abs(w) < mMinRollSpeed ) {
                w = (w > 0) ? mMinRollSpeed : -mMinRollSpeed;
            }      /// Set Belt roll speed
		}
		PLOG_ERROR(ALGO_UTIL_TAG,"roll(angle: %f,w: %f,timeout: %d,error: %f) \n",angle,w,timeout,error);
		///PLOG_INFO(ALGO_UTIL_TAG,"Footprint(000D_...) \n");
        DistanceCloud dist;
        return rollAndGenDistance(angle,w,timeout,error,false,dist);
    }

    int AlgoUtils::rollAndFindPile(float angle, float w, int timeout, float error){
        if (PltConfig::getInstance()->isTrackModel()) /// Track model ...
        {
            mMinRollSpeed = TRACKTYPE_MIN_ROLL_SPEED;   /// min roll speed for belt ...
        } else {
            mMinRollSpeed = MIN_ROLL_SPEED;
        }
        if(abs(w) < mMinRollSpeed ) {
            w = (w > 0) ? mMinRollSpeed : -mMinRollSpeed;
        }
        DistanceCloud dist;
        return rollAndGenDistanceEx(angle, w, mMinRollSpeed, timeout, DGREE_3_RAD,
                                    false, dist, true);
    }

    int AlgoUtils::rollEx(float angle,float w,int timeout,float error)
    {
        if (PltConfig::getInstance()->isTrackModel()) /// Track model ...
		{
			mMinRollSpeed = TRACKTYPE_MIN_ROLL_SPEED;   /// min roll speed for belt ...
		} else {
            mMinRollSpeed = MIN_ROLL_SPEED;
        }
		///PLOG_INFO(ALGO_UTIL_TAG,"Footprint(000E_...) \n");
        return rollEx(angle, w, mMinRollSpeed, timeout, error);
        // DistanceCloud dist;
        // return rollAndGenDistanceEx(angle,w,timeout,error,false,dist);
    }


    int AlgoUtils::rollEx(float angle,float w, float minW, int timeout,float error)
    {
        DistanceCloud dist;
		PLOG_INFO(ALGO_UTIL_TAG,"rollEx(angle: %f,w: %f,minW: %f,timeout: %d,error: %f) \n",angle,w,minW,timeout,error);
		///PLOG_INFO(ALGO_UTIL_TAG,"Footprint(000F_...) \n");
        return rollAndGenDistanceEx(angle, w, minW, timeout,error,false,dist);
    }

    int AlgoUtils::rollAndGenDistance(float angle,float w,int timeout,float error,bool genDist,DistanceCloud& dist)
    {
        int time,ret=0,dur;
        float rolled,err,wz;
        if (PltConfig::getInstance()->isTrackModel()) /// Track model ...
		{
			mMinRollSpeed = TRACKTYPE_MIN_ROLL_SPEED;   /// min roll speed for belt ...
		} else {
            mMinRollSpeed = MIN_ROLL_SPEED;
        }
        float minRollSpeed = (PltConfig::getInstance()->isTrackModel() && genDist) ?
                            TRACK_GEN_DISTANCE_SPEED : mMinRollSpeed;

		///PLOG_INFO(ALGO_UTIL_TAG,"Footprint(0010_...) \n");
		PLOG_INFO(ALGO_UTIL_TAG,"rollAndGenDistance(angle: %f,w: %f,timeout: %d,error: %f,genDist: %d,dist: %p) \n",angle,w,timeout,error,genDist,dist);

        //// ROS_INFO("rollAndGenDistance mQuitFlag: %d angle:%f  w:%f %s %d \n", mQuitFlag, angle, w, __FILE__, __LINE__);
        if(abs(w) < minRollSpeed){
            w = (w>0) ? minRollSpeed : -minRollSpeed;
        }

        return rollAndGenDistanceEx(angle, w, minRollSpeed, timeout, error, genDist, dist);
    }

    int AlgoUtils::rollAndGenDistanceRm(float angle,float w,int timeout,float error,bool genDist,shared_ptr<DistanceMap> distMap)
    {
        int time,ret=0,dur;
        double rolled,err,wz=0.0;
        float minW = PltConfig::getInstance()->isTrackModel() ?
                            TRACK_GEN_DISTANCE_SPEED : MIN_GEN_DISTANCE_SPEED;

		PLOG_INFO(ALGO_UTIL_TAG,"rollAndGenDistanceRm w: %f, minW: %f \n", w, minW);
        if(abs(angle)<=0.025){
          return 0;
        }

        if(genDist){
            mDistance=FLT_MAX;
            mIsFirstTof = true;
            mTof=mHandle.subscribe("SensorNode/tof",1,&AlgoUtils::onTofData,this);
        }
        while(1){
			if(mIsFirstTof){
				usleep(IMU_WATING_TIME*1000);
				time+=IMU_WATING_TIME;
				continue;
			}else{
				break;
			}
        }

        moveOnce(0.0,0.0,0.0,1000,0);
        if(abs(w) < minW){
            w = (w>0) ? minW : -minW;
        }

        mRollAngular=0.0;
        mIsFirstIMU=true;
        mIMUCalibraCnt=0;
        mZCalibra=0.0;
        imuCallbackType_ = GET_YAW;
        mIMU=mHandle.subscribe("SensorNode/imu",1000,&AlgoUtils::onIMUData,this);

        time=0;

        DeviceDefaultConfig cfg;
        float KP = cfg.getRollExKp();
        float KI = cfg.getRollExKi();
        float KD = cfg.getRollExKd();
        // float MaxInitSpeed = cfg.getRollExMaxInitialSpeed();
        float err_wt_ = 0.0;
        float total_err_w_ = 0.0;
        float proportional_ = 0.0;
        float integral_ = 0.0;
        float derivative_ = 0.0;
        float dt = 0.0;
        auto pre_time = ros::Time::now();
        float pre_err = 0.0;
        float pre_rolled = 0.0;
        float pre_wz = 0.0;
        // PLOG_DEBUG(ALGO_UTIL_TAG, "[Roll PI] KP: %f, KI: %f, KD: %f", KP, KI, KD);

        PLOG_DEBUG(NAV_PATH_NODE_TAG, "Starting roll and gen dist");
        while(1){
            if(mQuitFlag ||
                 (timeout>=0 && time>timeout)){
                ret=-1;
                break;
            }
            if(mIsFirstIMU){
                usleep(2*IMU_WATING_TIME*1000);
                time+=2*IMU_WATING_TIME;
                pre_time = ros::Time::now();
                continue;
            }
            rolled = mRollAngular;
            if(genDist){
            	if(FLT_MAX != mDistance){
					pair<double, double> DistPoint(rolled, mDistance);
					distMap->push_back(DistPoint);
                    if(mDistance > FREE_SPACE_DIST && abs(rolled) > 0.05 /*ZERO_ANGLE_THRESHOLD*/){
                        PLOG_INFO(ALGO_UTIL_TAG, "Stop roll for gen dist angle: %f, dist: %f", rolled, mDistance);
                        ret = 1;
                        break;
                    }
            	}
            }
            err=abs(rolled-angle);
            if(err<error){
                ret = 0;
                break;
            }

            ///< PID vel
            dt = (ros::Time::now() - pre_time).toSec();
            pre_time = ros::Time::now();

            if(pre_wz != 0.0){
                err_wt_ = pre_wz - (rolled - pre_rolled)/dt;
                proportional_ = KP * err_wt_;

                total_err_w_ += err_wt_ * dt;
                integral_ = KI * total_err_w_;
                integral_ = (integral_ < w) ? integral_ : w;
                if (pre_err != 0.0){
                    derivative_ = KD * (err_wt_ - pre_err)/dt;
                }
            }
            // integral_ = (integral_ < w) ? integral_ : w;
            float pid = proportional_ + integral_ + derivative_;

            wz = err/abs(angle)*(w-minW) + minW;
            wz *= rolled>angle?-1:1;
            wz += pid;

            if (abs(wz) < minW) {
                wz = (wz < 0) ? -minW : minW;
            }
            else if (abs(wz) > (w + MAX_ROLL_SPEED)) {
                wz = (wz < 0) ? -(w + MAX_ROLL_SPEED) : (w + MAX_ROLL_SPEED);
            }
            pre_wz = wz;
            pre_rolled = rolled;
            pre_err = err_wt_;
            // dur = ROLL_DURATION;
            dur = 10;

            moveOnce(0.0,0.0,wz,dur,0);
            time+=dur;
        }
        moveOnce(0.0,0.0,0.0,0,50);
        mIMU.shutdown();
        if(genDist){
            mTof.shutdown();
        }
        resetQuitFlag();
        return ret;
    }

    int AlgoUtils::rollAndGenDistanceEx(float angle,float w,int timeout,float error,bool genDist,DistanceCloud& dist)
    {
		PLOG_INFO(ALGO_UTIL_TAG,"Footprint(0012_...) \n");
        return rollAndGenDistanceEx(angle, w, mMinRollSpeed, timeout, error, genDist, dist);
    }

    int AlgoUtils::rollAndGenDistanceEx(float angle,float w,float minW, int timeout,float error,bool genDist,DistanceCloud& dist, bool findPile)
    {
        int time,ret=0,dur;
        float rolled,err,wz;

		PLOG_INFO(ALGO_UTIL_TAG,"rollAndGenDistanceEx PID(angle: %f,w: %f,minW: %f,timeout: %d,error: %f,genDist: %d,dist: %p) \n",angle,w,minW,timeout,error,genDist,dist);
		///PLOG_INFO(ALGO_UTIL_TAG,"Footprint(0013_...) \n");
        if(abs(angle)<=0.025){
          return 0;
        }

        moveOnce(0.0,0.0,0.0,1000,0);
        if(abs(w)<minW){
            w = (w>0) ? minW : -minW;
        }

        mRollAngular=0.0;
        mIsFirstIMU=true;
        mIMUCalibraCnt=0;
        mZCalibra=0.0;
        if(!mIMU){
            // mIMU=mHandle.subscribe("SensorNode/imu",1000,&AlgoUtils::onIMUData,this);
            t_last_imu = ros::Time::now();
            cnt = 0;
            imuCallbackType_ = GET_YAW;
            mIMU = mHandle.subscribe(imuSubOps);
        }
        if(genDist){
            mDistance=FLT_MAX;
            mTof=mHandle.subscribe("SensorNode/tof",1,&AlgoUtils::onTofData,this);
        }
        time=0;

        int tries = 0;
        int maxTries = timeout/ROLL_DURATION;
        DeviceDefaultConfig cfg;
        float KP = cfg.getRollExKp();
        float KI = cfg.getRollExKi();
        float KD = cfg.getRollExKd();
        // float MaxInitSpeed = cfg.getRollExMaxInitialSpeed();
        float err_wt_ = 0.0;
        float total_err_w_ = 0.0;
        float proportional_ = 0.0;
        float integral_ = 0.0;
        float derivative_ = 0.0;
        float dt = 0.0;
        auto pre_time = ros::Time::now();
        float pre_err = 0.0;
        float pre_rolled = 0.0;
        float pre_wz = 0.0;
        // PLOG_DEBUG(ALGO_UTIL_TAG, "[Roll PI] KP: %f, KI: %f, KD: %f", KP, KI, KD);
        float sum_wz_ = 0.0;
        int   tmp_cnt_ = 0;

        // auto break_time = ros::Time::now();
        mCharging = -1;
        if(findPile && !mBatteryStatus){
            mBatteryStatus = mHandle.subscribe("SensorNode/simple_battery_status", 10, &AlgoUtils::onBatteryStatus, this);
        }
        while(1){
            ros::spinOnce();
            if(mQuitFlag ||
                 //(maxTries>0 && tries>=maxTries)){
                (timeout>=0 && time>timeout)){
                ret=-1;
                break;
            }
            if(findPile && (mCharging > 0)){
                PLOG_INFO(ALGO_UTIL_TAG, "Detect charging during rotate");
                ret = 1;
                break;
            }
#ifdef SENSOR_IMU_USE_CALIBRA
            if(mIMUCalibraCnt < ROLL_IMU_CALIBRA_CNT){
                continue;
            }
#endif
            if(mIsFirstIMU){
                usleep(IMU_WATING_TIME*1000);
                time+=IMU_WATING_TIME;
                pre_time = ros::Time::now();
                continue;
            }
            // mAngleMutex.lock();
            rolled=mRollAngular;
            // mAngleMutex.unlock();
            if(genDist){
                dist.push_back(rolled);
                dist.push_back(mDistance);
            }
            err = abs(rolled-angle);
            if(abs(err)<error){
                break;
            }

            ///< PID vel
            dt = (ros::Time::now() - pre_time).toSec();
            pre_time = ros::Time::now();

            if(pre_wz != 0.0){
                err_wt_ = pre_wz - (rolled - pre_rolled)/dt;
                proportional_ = KP * err_wt_;

                total_err_w_ += err_wt_ * dt;
                integral_ = KI * total_err_w_;
                integral_ = (integral_ < w) ? integral_ : w;
                if (pre_err != 0.0){
                    derivative_ = KD * (err_wt_ - pre_err)/dt;
                }
            }
            // integral_ = (integral_ < w) ? integral_ : w;
            float pid = proportional_ + integral_ + derivative_;

            wz = err/abs(angle)*(w-minW) + minW;
            wz *= rolled>angle?-1:1;
            wz += pid;

            if (abs(wz) < minW) {
                wz = (wz < 0) ? -minW : minW;
            }
            else if (abs(wz) > (w + MAX_ROLL_SPEED)) {
                wz = (wz < 0) ? -(w + MAX_ROLL_SPEED) : (w + MAX_ROLL_SPEED);
            }
            // PLOG_DEBUG(ALGO_UTIL_TAG, "[Roll PI] setpoint: %f, curr: %f, err: %f, P_term: %f, I_term: %f, D_term: %f, wz: %f, dt: %f", pre_wz, (rolled-pre_rolled)/dt, err_wt_, proportional_, integral_, derivative_, wz, dt);
            pre_wz = wz;
            pre_rolled = rolled;
            pre_err = err_wt_;
            dur = 10;

            sum_wz_ += wz;
            tmp_cnt_++;
            moveOnce(0.0,0.0,wz,dur,0);
            //tries++;
            time+=dur;
        }
        if(tmp_cnt_ > 0) {
            mAvgWz = sum_wz_/(float)tmp_cnt_;
        }
        PLOG_DEBUG(ALGO_UTIL_TAG, "[Rotate done] rolled: %f, average wz: %f", rolled, mAvgWz);
        moveOnce(0.0,0.0,0.0,0,50);
        // mAngleMutex.lock();
        rolled = mRollAngular;
        // mAngleMutex.unlock();
        mIMU.shutdown();
        if(genDist){
                mTof.shutdown();
        }
        if (findPile && mBatteryStatus){
            mBatteryStatus.shutdown();
        }
        resetQuitFlag();
        return ret;
    }

    int AlgoUtils::genDistance(DistanceCloud& dist)
    {
		PLOG_INFO(ALGO_UTIL_TAG,"Footprint(0014_...) \n");
        dist.resize(0);
        return rollAndGenDistance(M_PI*2,GEN_DISTANCE_SPEED,DURATION_10_S,DGREE_3_RAD,true,dist);
    }

    int AlgoUtils::getDistance(float angle,shared_ptr<DistanceMap> distMap)
    {
		PLOG_INFO(ALGO_UTIL_TAG,"Footprint(0015_...) \n");
        //dist.resize(0);
        distMap->clear();
        return rollAndGenDistanceRm(angle,GEN_DISTANCE_SPEED,DURATION_10_S,DGREE_3_RAD,true,distMap);
    }

    float AlgoUtils::getMaxSpace(DistanceCloud& dist,float distThreshold,float& arc)
    {
        float maxAngle=-1.0;
        float maxIdx=-1.0;
        float startIdx=-1.0;
        float angle;

		PLOG_INFO(ALGO_UTIL_TAG,"Footprint(0016_...) \n");

        for(int i=0;i<(int)dist.size();i+=2){
            if(dist[i]<0){
                dist[i]=0.0;
            }
            if(dist[i+1]>distThreshold){
                if(startIdx<0){
                    startIdx=dist[i];
                }
                if(i+2!=(int)dist.size()){
                    continue;
                }
            }
            if(startIdx<0){
                continue;
            }

            angle=dist[i]-startIdx;
            if(angle>maxAngle){
                maxIdx=startIdx;
                maxAngle=angle;
            }
            startIdx=-1.0;
        }
        arc=maxAngle;
        return maxIdx;
    }

    int AlgoUtils::move(float x,float y,float speed,int timeout,float error)
    {
        float s=sqrt(x*x+y*y);
        const float speed_e=1.0;

		PLOG_INFO(ALGO_UTIL_TAG,"move(x: %f,y: %f,speed: %f,timeout: %d,error: %f) \n",x,y,speed,timeout,error);

        if(s<1e-3){
		PLOG_INFO(ALGO_UTIL_TAG,"Footprint(0018_Distance too short!) \n");
            return 0;
        }
        float speedX=speed*x/s;
        float speedY=speed*y/s;
        int duration=std::max(abs(int(s/speed*1000/speed_e)),250);
        if (PltConfig::getInstance()->isTrackModel())  /// Track model ...
        {
            if (y > 0)
              duration = duration*mMoveForwardRatio;
            else
              duration = duration*mMoveBackwardRatio;
        }
        moveOnce(speedX,speedY,0.0,duration,1);
        return 0;
    }

    int AlgoUtils::moveEx(float x,float y,float speed,int timeout,float error)
    {
		PLOG_INFO(ALGO_UTIL_TAG,"moveEx(x: %f,y: %f,speed: %f,timeout: %d,error: %f) \n",x,y,speed,timeout,error);
        float s=sqrt(x*x+y*y);
        const float speed_e=1.0;
        if(s<1e-3){
            return 0;
        }
        float speedX=speed*x/s;
        float speedY=speed*y/s;
        int duration=std::max(int(s/speed*1000/speed_e),250);
        moveOnceEx(speedX,speedY,0.0, s, 2*duration,1);
        return 0;
    }


    /**
     * @description:
     * @param speedX m/s
     * @param speedY m/s
     * @param speedW rad/s
     * @param time in seconds
     * @return {*}
     */
    int AlgoUtils::action(float speedX,float speedY,float speedW,int time)
    {
		PLOG_INFO(ALGO_UTIL_TAG,"Footprint(001A_...) \n");
        moveOnce(speedX,speedY,speedW,time,1);
        return 0;
    }

    void AlgoUtils::onBatteryStatus(const statusConstPtr &s)
    {
		///<<< PLOG_INFO(ALGO_UTIL_TAG,"Footprint(001B_...) \n");
		// ROS_INFO("Get BattStatus: %x \n", s );
		PLOG_INFO(ALGO_UTIL_TAG,"Get BattStatus: %X \n", s->status);
        mCharging=s->status[2];
    }

    /**
     * @description: For mecanum
     * @param {*}
     * @return {*}
     */
    int AlgoUtils::connectToChargePile(int tryCnt, int round)
    {
        bool bFs8003Motor = false;

		PLOG_INFO(ALGO_UTIL_TAG,"Footprint(001C_...) \n");

        if (PltConfig::getInstance()->isChinaMotor()){    /// Fix motor type
            bFs8003Motor = true;
		    PLOG_INFO(ALGO_UTIL_TAG,"HWconfig is China motors !\n ");
        } else {
		    PLOG_INFO(ALGO_UTIL_TAG,"HWconfig is ST motors !\n ");
        }

        int t=0;
        int check_charging = 0;
        mCharging=-1;
        mBatteryStatus=mHandle.subscribe("SensorNode/simple_battery_status",10,&AlgoUtils::onBatteryStatus,this);

        const int _ms100=100*1000;
        //while(mCharging<0 && mIsFirstIMU){
        while(mCharging<0 ){
            usleep(_ms100);
            t+=_ms100;
            // if(t>500000){
            if(t>10 * _ms100){
                break;
            }
            if( mQuitFlag){
				PLOG_INFO(ALGO_UTIL_TAG,"Return -1 on mQuitFlag == true !\n ");
                return -1;
            }
        }
        if(checkCharging() == 1){
            mBatteryStatus.shutdown();
            PLOG_INFO(ALGO_UTIL_TAG,"Return 0 on mCharging == true (%d) !\n ",mCharging);
            return 0  ;
        }

        float vx = mCfg.getBackupVx();
        float vy = mCfg.getBackupVy();
        float wz = mCfg.getBackupWz();

        if (bFs8003Motor){
            vy *= 1.1 ; ///<<< Roger tune for Fs8003 motor 1.4;
	        if (vx < 0) vx*= 0.5;
        }
        t = 0;
        int sleepTm = 100;
        int duration = mCfg.getBackupDuration();

        while(!mQuitFlag){
            //moveOnce(0,vy,0,CONNECT_CHARGE_PILE_ONCE_TIME,1000);
            PLOG_INFO(ALGO_UTIL_TAG,"Try to docking...(%d: %f , %f, %f) !\n ",t,vx,vy,wz);
            if ( t != (tryCnt/2)){
                moveOnce(vx,vy,wz,duration,1000);
            } else {
                moveOnce(0,vy,0,duration,1000);
            }

            for (int i=0; i<10; i++){
                usleep(sleepTm*1000);
                //if (mCharging ||  mAccZ<(G_NORMAL-0.15)){
                if (mCharging > 0 ){
                    break;
                }
                if (mQuitFlag){
                    break;
                }
            }

            if(checkCharging() > 0) {
                break;
            }

            if(++t>tryCnt){
                break;
            }

            if (round == 0){
                if (t == (tryCnt/2)){
                    vx *= -1;
                    if(bFs8003Motor && vx < 0) {
                        vx *= 0.75;
                        vy *= 0.85;
                    }
                }
            } else {
                if (t == (tryCnt/2))
                {
                    vx *= -1;  ///<<< reverse adjust direction
                    if (0.1 > abs(mCfg.getBackupWz())){
                        if(vx < 0.0) {
                            wz = MIN_ROLL_SPEED + (round+1.0)*0.1;
                            if( wz > MAX_BACKUP_ROLL_SPEED )
                                wz = MAX_BACKUP_ROLL_SPEED;
                            vx *= (1.0 + round*0.1);
                            if (vx < -MAX_BACKUP_SHIFT_SPEED)
                                vx = -MAX_BACKUP_SHIFT_SPEED;
                        }
                    }
                }
                else if (t < (tryCnt/2) && round < 3) {
                    wz = min(MAX_BACKUP_ROLL_SPEED,float( MIN_ROLL_SPEED + (round+1.0)*0.1));
                    moveOnce(0,0,-wz,300,1000);
                    usleep(100*1000);
                    wz = mCfg.getBackupWz();
                    if (mCharging > 0){
                        sleep(2); /// wait to confirm charge state
                        if	( true == mCharging )
                        {
                            PLOG_INFO(ALGO_UTIL_TAG," break 004 while on mCharging == true (try:%d) !\n ",t);
                            break ;
                        }
                    }
                }
                else if (t > (tryCnt/2) && round < 3) {
                    wz = min(MAX_BACKUP_ROLL_SPEED,float( MIN_ROLL_SPEED + (round+1.0)*0.1));
                    moveOnce(0,0,wz,300,1000);
                    usleep(100*1000);
                    wz = mCfg.getBackupWz();
                    if (mCharging > 0){
                        sleep(2); /// wait to confirm charge state
                        if	( true == mCharging )
                        {
                            PLOG_INFO(ALGO_UTIL_TAG," break 005 while on mCharging == true (try:%d) !\n ",t);
                            break ;
                        }
                    }
                }
            }
		//t++;
            //t+=CONNECT_CHARGE_PILE_ONCE_TIME;
        }

        usleep(1000*1000);
        mBatteryStatus.shutdown();
        resetQuitFlag();
        if (mCharging<=0){
			PLOG_INFO(ALGO_UTIL_TAG,"Return -1 on mCharging == -1 !\n ");
            return -1;
        }
        return 0;    //success !!!
    }


    /**
     * @description: For track model
     * @param {*}
     * @return {*}
     */
    int AlgoUtils::connectToChargePileTrack(int tryCnt)
    {
        bool bFs8003Motor = false;

		PLOG_INFO(ALGO_UTIL_TAG,"Footprint(001C_...) \n");

        if (PltConfig::getInstance()->isChinaMotor()){    /// Fix motor type
            	bFs8003Motor = true;
		    PLOG_INFO(ALGO_UTIL_TAG,"HWconfig is China motors !\n ");
        } else {
		    PLOG_INFO(ALGO_UTIL_TAG,"HWconfig is ST motors !\n ");
        }

        float vx = mCfg.getTrackTypeBackupVx();
        float vy = mCfg.getTrackTypeBackupVy();
        float wz = mCfg.getTrackTypeBackupWz();
        float swing_ratio = mCfg.getTrackTypeSwingRatio();
        int t=0;
        int t_climb = 0;
        int check_charging = 0;
        int swing_cnt = 0;
        float last_climb_wz;
        bool  just_climbed = false;
        float swing_wz = wz + (mAvgWz - 2.9)*swing_ratio;

        mCharging=-1;
        mBatteryStatus=mHandle.subscribe("SensorNode/simple_battery_status",10,&AlgoUtils::onBatteryStatus,this);

#ifdef USE_COMMON_IMU_CB
        mRollAngular=0.0;
        mIsFirstIMU = true;
#else
        mPitchAngular = 0.0;
        mIsPitchFirstIMU = true;
#endif
        // DeviceDefaultConfig cfg;
        mGyroxClimbingThres = mCfg.getTrackTypeGyroxClimbingThres();
        float xAngle = 0.0;
        if(mIMU) {
            mIMU.shutdown();
        }
        if (!mIMU){
#ifdef USE_COMMON_IMU_CB
            imuCallbackType_ = GET_PITCH;
            mIMU=mHandle.subscribe("SensorNode/imu",1000,&AlgoUtils::onIMUData,this);
#else
            mIMU=mHandle.subscribe("SensorNode/imu",1000,&AlgoUtils::onIMUData2,this);
#endif
        }
        const int _ms100=100*1000;
        //while(mCharging<0 && mIsFirstIMU){
        while(mCharging<0 ){
            usleep(_ms100);
            t+=_ms100;
            // if(t>500000){
            if(t>10 * _ms100){
                break;
            }
            if( mQuitFlag){
				PLOG_INFO(ALGO_UTIL_TAG,"Return -1 on mQuitFlag == true !\n ");
                if (mIMU){
                    mAngleMutex.lock();
#ifdef USE_COMMON_IMU_CB
                    mRollAngular = 0.0;
#else
                    mPitchAngular = 0.0;
#endif
                    mAngleMutex.unlock();
                    PLOG_DEBUG(ALGO_UTIL_TAG, "kill IMU");
                    mIMU.shutdown();
                }
                return -1;
            }
        }
        if(checkCharging() == 1){
            mBatteryStatus.shutdown();
            PLOG_INFO(ALGO_UTIL_TAG,"Return 0 on mCharging == true (%d) !\n ",mCharging);
            if (mIMU){
                mAngleMutex.lock();
#ifdef USE_COMMON_IMU_CB
                mRollAngular = 0.0;
#else
                mPitchAngular = 0.0;
#endif
                mAngleMutex.unlock();
                PLOG_DEBUG(ALGO_UTIL_TAG, "kill IMU");
                mIMU.shutdown();
            }
            return 0  ;
        }


        if (bFs8003Motor){
            vy *= 1.1 ; ///<<< Roger tune for Fs8003 motor 1.4;
	        if (vx < 0) vx*= 0.5;
        }
        t = 0;
        int sleepTm = 100;
        int duration = mCfg.getBackupDuration();

        while(!mQuitFlag){
            //moveOnce(0,vy,0,CONNECT_CHARGE_PILE_ONCE_TIME,1000);
            PLOG_INFO(ALGO_UTIL_TAG,"Try to docking...(%d: %f , %f, %f) !\n ",t,vx,vy,wz);
            if(shouldMoveOutWhenClimb(vy, swing_wz, t_climb, last_climb_wz, just_climbed)){
                duration = mCfg.getBackupDuration() * 0.8;
                swing_cnt++;
            }
            moveOnceForce(0,vy,0,duration,1000);
            for (int i=0; i<10; i++){
                usleep(sleepTm*1000);
                //if (mCharging ||  mAccZ<(G_NORMAL-0.15)){
                if (mCharging > 0 ){
                    break;
                }
                if (mQuitFlag){
                    break;
                }
            }

            if(checkCharging() > 0) {
                break;
            }

            if(shouldMoveOutWhenClimb(vy, swing_wz, t_climb, last_climb_wz, just_climbed)){
                duration = mCfg.getBackupDuration() * 0.8;
                swing_cnt++;
            }
            else {
                if(tryToContactChargePile(vy)){
                    break;
                }
            }

            if(++t>tryCnt){
                break;
            }
            if((!just_climbed) && (t>1)) {
                if(swing_cnt++ % 2 == 0){
                    swing_wz = (-1) * swing_wz;
                }
                duration = mCfg.getBackupDuration() / 2.0;
                PLOG_DEBUG(ALGO_UTIL_TAG, "swing wz=%f, swing_cnt: %i", swing_wz, swing_cnt);
                moveOnceForce(0, 0, swing_wz, 450, 1000);
                usleep(100*1000);
                check_charging = checkCharging();
                if(check_charging == 1){
                    PLOG_INFO(ALGO_UTIL_TAG," break 004 while on mCharging == true (try:%d) !\n ",t);
                    break ;
                } else if (check_charging == -1) {
                    if (tryToContactChargePile(vy)) break;
                }
                if(t % 3 == 0) {
                    PLOG_DEBUG(ALGO_UTIL_TAG, "Move out a little bit after many tries");
                    moveOnceForce(0, -vy, 0, 400, 1000);
                    mAngleMutex.lock();
#ifdef USE_COMMON_IMU_CB
                    mRollAngular = 0.0;
#else
                    mPitchAngular = 0.0;
#endif
                    mAngleMutex.unlock();
                }
            }
            if (just_climbed) {
                just_climbed = false;
                t--;
            }
        }
        if (mIMU){
            mAngleMutex.lock();
#ifdef USE_COMMON_IMU_CB
            mRollAngular = 0.0;
#else
            mPitchAngular = 0.0;
#endif
            mAngleMutex.unlock();
            PLOG_DEBUG(ALGO_UTIL_TAG, "kill IMU");
            mIMU.shutdown();
        }

        usleep(1000*1000);
        mBatteryStatus.shutdown();
        resetQuitFlag();
        if (mCharging<=0){
			PLOG_INFO(ALGO_UTIL_TAG,"Return -1 on mCharging == -1 !\n ");
			return -1;
		}
		return 0;	 //success !!!
	}

    bool AlgoUtils::isCharging()
    {
		PLOG_INFO(ALGO_UTIL_TAG,"Footprint(001D_...) \n");

        std::ifstream chargeFile("/sys/class/power_supply/rk-bat/status");
        string charge;
        chargeFile>>charge;
        if(chargeFile.fail()){
            return false;
        }

        if(charge == "Charging" || charge == "Full"){
           return true;
        }

        return false;
    }

    int AlgoUtils::reConnectToChargePile(int timeout)
    {
        int t=0;
        int sleepTm = 50;
        int connectOnceTime = 100;

		PLOG_INFO(ALGO_UTIL_TAG,"Footprint(001E_...) \n");

        while(!mQuitFlag){
            moveOnce2(0,-0.05,0,connectOnceTime,1000);

            for (int i=0; i<100; i++){
                usleep(sleepTm*1000);
                if (isCharging()){
                    break;
                }
            }
            if(t>timeout || isCharging()){
                break;
            }
            t+=connectOnceTime;
        }

        usleep(100*1000);
        mBatteryStatus.shutdown();
        resetQuitFlag();
        if (!isCharging()){
            return -1;
        }
        return 0;
    }

    void AlgoUtils::onObjdetect(const detectConstPtr& obj)
    {
		PLOG_INFO(ALGO_UTIL_TAG,"Footprint(001F_...) \n");

        lock_guard<mutex> lock(mObjMutex);
        if(obj->name==mObj){
            mHasDetected = true;
            if(mStartWaitObjStamp<obj->stamp){
                mDetected=true;

                float hScale = float(ALIGN_IMG_HEIGHT)/obj->height;
                float wScale = float(ALIGN_IMG_WIDTH)/obj->width;

                mPos.width=wScale * obj->width;
                mPos.height= hScale * obj->height;
                mPos.top=hScale * obj->top;
                mPos.left=wScale * obj->left;
                mPos.bottom=hScale*obj->bottom;
                mPos.right=wScale * obj->right;
                mPos.stamp = obj->stamp;
                mPos.detectStamp = mStartWaitObjStamp;
                // ROS_INFO("mStartWaitObjStamp:%ld,obj->stamp:%ld, hScale:%f wScale:%f,"
                    //   ,mStartWaitOb?jStamp, obj->stamp, hScale, wScale);
            }
        }
    }
    void AlgoUtils::onIMUData(const sensor_msgs::ImuConstPtr& imu)
    {
        // Add IMU calibration reference from nav_path_node
        int N_short =2 , N_long = 128;
        double Vdc_residue_limit = 0.002;
        static double avgDC_short = 0.0;
        static double avgDC_long = 0.0;
        static double prev_data = 0.0;
        double delta_GyroZ = 0.0;

        double imu_data;
        switch (imuCallbackType_)
        {
        case GET_YAW:
            imu_data = imu->angular_velocity.z;
            break;
        case GET_PITCH:
            imu_data = imu->angular_velocity.x;
            break;
        default:
            break;
        }

		///<<< PLOG_INFO(ALGO_UTIL_TAG,"Footprint(0020_...) \n");
		if(mIMUCalibraCnt<ROLL_IMU_CALIBRA_CNT){
            mZCalibra += imu_data;
            if(++mIMUCalibraCnt==ROLL_IMU_CALIBRA_CNT){
                mZCalibra/=ROLL_IMU_CALIBRA_CNT;
                PLOG_DEBUG(ALGO_UTIL_TAG, "imu calibration: mZCalibra=%f", mZCalibra);
            }
            avgDC_short =0.0;
            avgDC_long = 0.0;
            prev_data = 0.0;
            return;
        }

        double ang_vel = imu_data - mZCalibra;
        double data = ang_vel;
        avgDC_short = avgDC_short - avgDC_short/N_short + data/N_short;
        delta_GyroZ = avgDC_short - (data - prev_data);
        prev_data = data;

        if (delta_GyroZ > Vdc_residue_limit)
            delta_GyroZ = Vdc_residue_limit;
        if (delta_GyroZ < -Vdc_residue_limit)
            delta_GyroZ = -Vdc_residue_limit;

        avgDC_long = avgDC_long - avgDC_long/N_long + delta_GyroZ/N_long;
        ang_vel = data - avgDC_long;

        if(mIsFirstIMU){
            mIsFirstIMU=false;
            mPreIMU=*imu;
            return;
        }

        auto dt=(imu->header.stamp-mPreIMU.header.stamp).toSec();
        mAngleMutex.lock();
        // mRollAngular+=((mPreIMU.angular_velocity.z+imu->angular_velocity.z)/2-mZCalibra)*t;
        mRollAngular += ang_vel *dt;
        mAngleMutex.unlock();
        mPreIMU=*imu;
    }

    void AlgoUtils::onIMUData2(const sensor_msgs::ImuConstPtr& imu)
    {
        if(mIsPitchFirstIMU){
            mIsPitchFirstIMU=false;
            mPrePitchIMU=*imu;
            return;
        }
        auto t=(imu->header.stamp-mPrePitchIMU.header.stamp).toSec();
        mAngleMutex.lock();
        mPitchAngular+=((mPrePitchIMU.angular_velocity.x+imu->angular_velocity.x)/2)*t;
        mAngleMutex.unlock();
        mPrePitchIMU=*imu;
        if ((ros::Time::now() - t_last_imu).toSec() > 0.1) {
            PLOG_DEBUG(ALGO_UTIL_TAG, "[HDEV] xAngle: %f", mPitchAngular);
            t_last_imu = ros::Time::now();
        }
    }

    void AlgoUtils::onTofData(const sensor_msgs::RangeConstPtr &r)
    {
		float range = r->range;
		if (isinf(r->range) || r->range <0){
			range = MAX_VALID_TOF_DIST;
		}
        mTofDataCnt = (mTofDataCnt < mTofAvgCnt) ? mTofDataCnt + 1 : mTofAvgCnt;
		mAvgTof = mAvgTof+(range-mAvgTof)/mTofDataCnt;
        // mAvgTof = range;
        mAngleMutex.lock();
		mDistance=mAvgTof;
		mIsFirstTof = false;
		// PLOG_INFO(ALGO_UTIL_TAG,"Footprint(0021_...) mDistance: %f\n", mDistance);
		//mDistance=r->range;
        mAngleMutex.unlock();
    }
     void AlgoUtils::magneticCalibrate(int timeout)
     {
		PLOG_INFO(ALGO_UTIL_TAG,"Footprint(0022_...) \n");

        const int DUR_MS=100;
        mMoveTime=0;
        mDataReady=false;
        mMagDatas.clear();
        mMagCalibraSub=mHandle.subscribe("SensorNode/mag",500,&AlgoUtils::onMagData,this);

        while(1){
            moveOnce(0.0,0.0,2.0,DUR_MS,0);
            mMoveTime+=DUR_MS;
            if(mDataReady){
                break;
            }
            if(mMoveTime>timeout){
                return;
            }
        }
        mMagCalibraSub.shutdown();
        moveOnce(0.0,0.0,0.0,DUR_MS,1000);
        if(mMagDatas.size()<100){
            return;
        }
        auto result=cv::fitEllipse(mMagDatas);
        double calibra[MAG_CALIBRA_PARAM_CNT];

        calibra[0]=result.center.x/MAG_CALIBRA_GAIN;//x offset
        calibra[1]=result.center.y/MAG_CALIBRA_GAIN;//y offset
        if(std::abs(result.angle)>45){
            calibra[2]=result.size.height/2;//x width
            calibra[3]=result.size.width/2;//y height
            result.angle=(result.angle>0?(result.angle-90):(result.angle+90));
        }else{
            calibra[2]=result.size.width/2;//x width
            calibra[3]=result.size.height/2;//y height

        }
        calibra[4]=-result.angle*M_PI/180;//angle

        //// ROS_INFO("center[%f,%f],size=[%f,%f],angle=%f",calibra[0],calibra[1],calibra[2],calibra[3],calibra[4]);
        saveCalibration(MAG_CALIBRATION_FILE,calibra,MAG_CALIBRA_PARAM_CNT);
     }
    void AlgoUtils::onMagData(const sensor_msgs::MagneticFieldConstPtr &mag)
    {
		PLOG_INFO(ALGO_UTIL_TAG,"Footprint(0023_...) \n");

        if(mMoveTime<3000||mDataReady){
            return;
        }
        mMagDatas.emplace_back(mag->magnetic_field.x*MAG_CALIBRA_GAIN,mag->magnetic_field.y*MAG_CALIBRA_GAIN);
        if(mMagDatas.size()>=1000){
            mDataReady=true;
        }
    }

    void AlgoUtils::odomRelativeCB(const nav_msgs::Odometry::ConstPtr& msg)
    {
		PLOG_INFO(ALGO_UTIL_TAG,"Footprint(0024_...) \n");

        Eigen::Vector3d t(msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);
        float vx=msg->twist.twist.linear.x;
        float vy=msg->twist.twist.linear.y;
        float w=msg->twist.twist.angular.z;

        mCurPostion+=mCurPose*t;
        mCurPose=mCurPose*Eigen::Quaterniond(msg->pose.pose.orientation.w,msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z);

    }

    void AlgoUtils::imuQueueThread()
    {
        static const double timeout = 0.01;
        while (mHandle.ok())
        {
            this->imuCallbackQueue.callAvailable(ros::WallDuration(timeout));
        }
    }

    bool AlgoUtils::shouldMoveOutWhenClimb(float vy, float wz, int& cnt, float& last_wz, bool& just_climbed)
    {
        mAngleMutex.lock();
#ifdef USE_COMMON_IMU_CB
        float angle = mRollAngular;
#else
        float angle = mPitchAngular;
#endif
        float wz_ = -wz;
        if (just_climbed && wz == last_wz) {
            wz_ = -wz_;
        }
        mAngleMutex.unlock();
        PLOG_DEBUG(ALGO_UTIL_TAG, "mPitchAngle: %f", angle);
        if(abs(angle) > mGyroxClimbingThres){
            PLOG_DEBUG(ALGO_UTIL_TAG, "move out on climbing");
            cnt++;
            moveOnceForce(0, -vy, 0, 400, 1000);
            usleep(100*1000);

            moveOnceForce(0, 0, wz_,400,1000);
            usleep(100*1000);

            moveOnceForce(0,-vy,0,600,1000);
            usleep(100*1000);

            just_climbed = true;
            last_wz = wz_;
            mAngleMutex.lock();
#ifdef USE_COMMON_IMU_CB
            mRollAngular = 0.0; // reset mPitch angular because of the error accumulate
#else
            mPitchAngular = 0.0;
#endif
            mAngleMutex.unlock();
            return true;
        }
        return false;
    }

    /* @return:
    *       -1: nearly charge
    *       1: fully charge
    *       0: not charge
    */

    int AlgoUtils::checkCharging()
    {
        if (mCharging > 0){
            sleep(2); /// wait to confirme charge state
            PLOG_DEBUG(ALGO_UTIL_TAG, "checkCharging: %i", mCharging ? 1: -1);
            return mCharging ? 1 : -1;
        }
        else {
            PLOG_DEBUG(ALGO_UTIL_TAG, "checkCharging: %i", mCharging);
            return 0;
        }
    }

    /* @return:
    *   -1: charging detected once
    *    0: Not charge
    *    1: Charged
    */
    bool AlgoUtils::tryToContactChargePile(float vy)
    {
        bool charging_detected_once{false};
        int check_charging = 0;
        bool first = true;
        int cnt = 0;

        while((first || charging_detected_once) && cnt++ < 5){
            first = false;
            PLOG_DEBUG(ALGO_UTIL_TAG, "try to contact %i", cnt);
            check_charging = checkCharging();
            if (check_charging == 1){
                return true;
            } else if (check_charging == -1) {
                charging_detected_once = true;
            }

            moveOnceForce(0,-vy,0,180,1000);
            usleep(100*1000);
            check_charging = checkCharging();
            if (check_charging == 1){
                return true;
            } else if (check_charging == -1) {
                charging_detected_once = true;
            }

            moveOnceForce(0,vy, 0,300,1000);
            usleep(100*1000);
            check_charging = checkCharging();
            if(check_charging == 1){
                return true;
            } else if (check_charging == -1) {
                charging_detected_once = true;
            }
        }
        return false;
    }

    void loadMagCalibraParam(MagCalibraParam& param)
    {
		PLOG_INFO(ALGO_UTIL_TAG,"Footprint(0025_...) \n");

        double calibra[MAG_CALIBRA_PARAM_CNT];
        if(!readCalibration(MAG_CALIBRATION_FILE,calibra,MAG_CALIBRA_PARAM_CNT)){
            calibra[0]=0.0;//x offset
            calibra[1]=0.0;//y offset
            calibra[2]=1.0;//x width
            calibra[3]=1.0;//y height
            calibra[4]=0.0;//angle
        }
        param.x=calibra[0];
        param.y=calibra[1];
        param.w=calibra[2];
        param.h=calibra[3];
        param.c=std::cos(calibra[4]);//cos(angle)
        param.s=std::sin(calibra[4]);//sin(angle)
        //// ROS_INFO("mag calibra:%f,%f,%f,%f,%f,%f",param.x,param.y,param.w,param.h,param.c,param.s);
    }
    void magCalibrate(double& x,double&y,double&z,MagCalibraParam& param)
    {
		PLOG_INFO(ALGO_UTIL_TAG,"Footprint(0026_...) \n");

        double x1=x-param.x;
        double y1=y-param.y;
        double avg=(param.w+param.h)/2;

        x=(x1*param.c-y1*param.s)/param.w*avg;
        y=(x1*param.s+y1*param.c)/param.h*avg;
    }
} // namespace roller_eye
