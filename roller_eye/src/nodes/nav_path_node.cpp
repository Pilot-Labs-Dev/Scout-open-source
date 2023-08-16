#include<memory>
#include<thread>
#include<atomic>
#include<boost/bind.hpp>
#include"ros/ros.h"
#include <ros/callback_queue.h>
#include <ros/spinner.h>
#include<nav_msgs/Odometry.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include"roller_eye/nav_path_save.h"
#include"roller_eye/nav_path_start.h"
#include"roller_eye/nav_patrol.h"
#include"roller_eye/nav_cancel.h"
#include"roller_eye/nav_list_path.h"
#include"roller_eye/nav_delete_path.h"
#include"roller_eye/nav_patrol_stop.h"
#include"roller_eye/nav_mag_calibra.h"
#include"roller_eye/nav_get_patrol_name.h"
#include"roller_eye/nav_get_status.h"
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Geometry>
#include"roller_eye/system_define.h"
#include"roller_eye/algo_utils.h"
#include"roller_eye/track_trace.h"
#include"roller_eye/plt_tools.h"
#include"roller_eye/plog_plus.h"
#include"roller_eye/util_class.h"
#include"roller_eye/status_publisher.h"
#include"roller_eye/ros_tools.h"
#include"roller_eye/plt_config.h"
#include "roller_eye/nav_trace_done.h"
#include "geometry_msgs/TwistStamped.h"
#include"imu_filter_madgwick/imu_filter.h"
#include"imu_filter_madgwick/stateless_orientation.h"
#include "roller_eye/sound_effects_mgr.h"
#include "roller_eye/patrol_status.h"
#include "roller_eye/detect_record_get_status.h"
#include "roller_eye/nav_waypoint_add.h"
#include "roller_eye/nav_waypoint_query.h"
#include "zlog.h"
#include "roller_eye/param_utils.h"
#include "roller_eye/enable_vio.h"
#include "roller_eye/imu_patrol_calib.h"
#include "roller_eye/getimu_patrolcalib_status.h"
#include "roller_eye/getDiffAngleWhenPatrol.h"
#include "roller_eye/saveTmpPicForStartPath.h"
#include "roller_eye/nav_calibration_get_status.h"
#include "roller_eye/nav_low_bat.h"
#include "roller_eye/nav_exit.h"

using namespace std;
using namespace roller_eye;
using namespace Eigen;
#include "imu_filter_madgwick/imu_filter_ros.h"
using namespace geometry_msgs;
using namespace sensor_msgs;
typedef sensor_msgs::Imu ImuMsg;
///<<< Use gyro.z * dt as increment of Yaw
#define	USE_SIMPLE_YAW_ESTIMATION	1
#define NO_IMU_FILTER_NODE	1

#define TEST_SAVE_FILE	0		///0	///1	///<<< 0 for disable file log    ///<<< define 1 if enabled
#define LOG_ARRAY_SIZE	(65536*2)
#define DETECT_MOVE_TIMEOUT     (5000)
#define DETECT_MOVEOUT_DISTANCE        (0.07)
#define DETECT_MOVE_SPEED       (0.15)
#define MAX_DISTANCE_TO_FIND_CHARGER    (1.2)

static const Vector3d G(0.0,0.0,G_VALUE);
const char tab = '\t', cr = '\r';

#if TEST_SAVE_FILE
ofstream imuLog;

TwistStamped logArray[LOG_ARRAY_SIZE];
int logIndex = 0, logCount =0;
int fileCount =0;
char logFileName[255];
#endif

#define NAV_PATH_NODE_TAG           "nav_path_node"
#define NAV_PATH_MAX_SIZE               100000

#define NAV_OBSTACLE_MIN_DIST               0.2
#define NAV_OBSTACLE_CNT                    5       // 5
#define NAV_OBSTACLE_MAX_DIST               0.5
#define NAV_MANUALSAVE_MIN_DIST             2.0          //min dist of manual save when schedule path
#define NAV_OBSTACLE_FORWARD_DIST           0.4
#define NAV_OBSTACLE_ANGLE                  (30.0*M_PI/180)
#define OBS_TOO_CLOSE_DIST                  0.1
#define OBS_MOVE_BACK_DIST                  0.1
#define OBS_MOVE_SPEED                      0.15
#define OBS_MOVE_FORWARD_DIST               0.4

#define AVOID_OBSTACLE_WITH_MUTEX	1    /// avoid process/thread conflict, RogerL 22/11/24
#define MAG_CALIBRATE_CNT               100
#define NAV_ERASE_CNT                           10

#define D_USE_IMU_FILTER_POSE     ///<<< Use IMU Filter pose instead of calc pose from motor odometry output

#define RETRUN_AVOID_IFCANCEL    if (mIsCancel) return;
#define RETURN_IFCHARGED if(mCharging) { return true;}

#define TRACK_ROLL_SPEED                    1.1
#define OBS_TRACK_ROLL_SPEED                1.7
#define TRACK_MOVE_SPEED                    0.15
#define TRACKTYPE_OBS_ROLL_SPEED            2.1

#define CALIB_MAX_WAIT_TIME                 5        ///<<< Max wait time for IMU calibration

static const string NAVIGATE_JPG_ROOT_PATH="/userdata/roller_eye/navigate_jpg/";
class NavigatePath{
#ifdef USE_MADGWICK_IMU_FILTER
  typedef sensor_msgs::Imu              ImuMsg;
  typedef sensor_msgs::MagneticField    MagMsg;
  typedef message_filters::sync_policies::ApproximateTime<ImuMsg, MagMsg> SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
  typedef message_filters::Subscriber<ImuMsg> ImuSubscriber;
  typedef message_filters::Subscriber<MagMsg> MagSubscriber;
#endif
public:
    NavigatePath():
    mLocal("~"),
    mDoingMagCalibra(false),
    mAdjusting(false),
    mPub(mLocal),
    mClipData(false),
    mIsPatroling(false),
    mDetectingPiling(false),
    mLowBatFlag(false),
    mIsStartFromOut(false),
    mDetectedPile(false),
    mPrevBatPercentage(100),
    mCurBatPercentage(0),
    mMaxPathSize(NAV_PATH_MAX_SIZE),
    mTofAvgCnt(4),
    mAvgTof(MAX_VALID_TOF_DIST),
    mIsCancel(false),
	mDoBacking(false),
    mDetectObstacleCnt(0),
	mTraceCounter(0),
	mImuFilterYaw(0.0),
	mGotImuYawOffset(false),
	mImuYawOffset(0.0),
	mSumW(0.0),
    mCharging(false)
    {
    	//rmwei
		log_t arg = {
			confpath:	"/var/roller_eye/config/log/" NAV_PATH_NODE_TAG ".cfg",
			levelpath:	"/var/roller_eye/config/log/" NAV_PATH_NODE_TAG ".level",			///<<<"log.level",
			logpath:	"/var/log/" NAV_PATH_NODE_TAG ".log",
			cname:		NAV_PATH_NODE_TAG
		};
		if(0 != dzlogInit(&arg,2)){
			printf("%s log int error.\r\n",NAV_PATH_NODE_TAG);
		}
		if(access(NAVIGATE_JPG_ROOT_PATH.c_str(),F_OK)!=0){
            mkdir(NAVIGATE_JPG_ROOT_PATH.c_str(),0755);
        }
		string strCmd = "sudo rm "+NAVIGATE_JPG_ROOT_PATH+"*_tmp";
		system(strCmd.c_str());
    	//rmwei end
        mMinBat = LOW_BATTERY_PER;
        PltConfig::getInstance()->getMinBat(mMinBat);

        VioParam param;
        load_vio_param(param);
        mBUseVio = param.enable;
        mk_depth_dir(NAVIGATE_PATH_PATH);
        mImuPatrolCalibClient = mLocal.serviceClient<imu_patrol_calib>("/imu_patrol_calib");
        mImuPatrolCalibStatusClient = mLocal.serviceClient<getimu_patrolcalib_status>("/getimu_patrolcalib_status");
        mImuFilterCalibStatusClient = mLocal.serviceClient<getimu_patrolcalib_status>("/imu/filter_calib_status");
        mDetectRecordStatusClient = mLocal.serviceClient<detect_record_get_status>("/DetectRecordNode/get_status");
		mSaveTmpPicClient = mLocal.serviceClient<saveTmpPicForStartPath>("/CoreNode/saveTmpPicForStartPath");
		mGetDiffAngleClient = mLocal.serviceClient<getDiffAngleWhenPatrol>("/CoreNode/getDiffAngleWhenPatrol");
		mImuFilterResetClient = mLocal.serviceClient<nav_cancel>("/imu/filter_reset");
		mImuFilterStartClient = mLocal.serviceClient<nav_cancel>("/imu/filter_start");
		mImuFilterStopClient = mLocal.serviceClient<nav_cancel>("/imu/filter_stop");

		mStart=mLocal.advertiseService("nav_path_start",&NavigatePath::start_path,this);
        mSave=mLocal.advertiseService("nav_path_save",&NavigatePath::save_path,this);
        mPatrol=mLocal.advertiseService("nav_patrol",&NavigatePath::patrol,this);
        mCancel=mLocal.advertiseService("nav_cancel",&NavigatePath::cancel,this);
        mPatrolStop=mLocal.advertiseService("nav_patrol_stop",&NavigatePath::stop_patrol,this);
        mListPath=mLocal.advertiseService("nav_list_path",&NavigatePath::list_path,this);
        mDeletePath=mLocal.advertiseService("nav_delete_path",&NavigatePath::delete_path,this);
        mMagClibra=mLocal.advertiseService("nav_mag_calibra",&NavigatePath::mag_calibra,this);
        mGetName=mLocal.advertiseService("nav_get_patrol_name",&NavigatePath::getName,this);
        mGetStatus=mLocal.advertiseService("nav_get_status",&NavigatePath::getStatus,this);
        mAddWayPt=mLocal.advertiseService("nav_waypoint_add",&NavigatePath::addWayPt,this);
        mQueryWayPt=mLocal.advertiseService("nav_waypoint_query",&NavigatePath::queryPt,this);
        mEnableVioSrv=mLocal.advertiseService("enable_vio",&NavigatePath::enableVio,this);
		mGetCalibStatusSrv=mLocal.advertiseService("nav_calibration_get_status",&NavigatePath::getCalibrationStatus,this);
		mLowBatSrv=mLocal.advertiseService("/nav_low_bat",&NavigatePath::lowBatCallback,this);
		mExitSrv=mLocal.advertiseService("nav_exit",&NavigatePath::navExit,this);

        mPose=mLocal.advertise<geometry_msgs::PoseStamped>("pose",100);
        mCmdVel=mGlobal.advertise<geometry_msgs::Twist>("cmd_vel",100);
        mNavTraceDonePub = mGlobal.advertise<std_msgs::Int32>("/nav_trace_done",1);
        mSysEvtPub = mGlobal.advertise<std_msgs::Int32>("/system_event",1);
        mPatrolStatusPub = mGlobal.advertise<patrol_status>("/patrol_status",1);
        mStatusPub = mGlobal.advertise<std_msgs::Int32>("CoreNode/going_home_status", 1);
        mTestNav = mGlobal.subscribe("/testNav",1,&NavigatePath::test,this);
        mBatteryStatus  = mGlobal.subscribe("SensorNode/simple_battery_status",
            10, &NavigatePath::batteryStatusCallback,this);
		mGoingHomeStatusSub = mGlobal.subscribe("CoreNode/going_home_status", 3,
            &NavigatePath::goingHomeStatus,this);

		mTimer=mGlobal.createTimer(ros::Duration(1),&NavigatePath::timerCallback,this);
		mTimer.stop();
        mObsWz = (PltConfig::getInstance()->isTrackModel()) ?
                    TRACKTYPE_OBS_ROLL_SPEED : OBS_TRACK_ROLL_SPEED;

        setPatrolStatus(false);

        PltConfig::getInstance()->getMaxNavSize(mMaxPathSize);
#ifdef USE_MADGWICK_IMU_FILTER
        mOrentianInited=false;
        mWorldFrame=WorldFrame::NWU;
        DeviceDefaultConfig cfg;
        mIMUFilterGain=cfg.getNavIMUFilterGain();
        mIMUFilterZeta=cfg.getNavIMUFilterZeta();
        mPubIMUPose=cfg.getNavPubIMUPose();
        if(mPubIMUPose){
            mIMUPose=mLocal.advertise<geometry_msgs::PoseStamped>("imu_pose",100);
        }
#endif
		fLog.open("/run/log/NavPath.log",ios::app);
		fLog << "Welcome to NavPath.log!" <<"@time  " <<ros::Time::now()<<fixed <<cr <<endl;

#ifdef IMU_USE_QUEUE_THREAD
        imuSubOps = ros::SubscribeOptions::create<sensor_msgs::Imu>(
          "/SensorNode/imu", // topic name
          100, // queue length
          boost::bind(&NavigatePath::imuCallback, this, _1), // callback
          ros::VoidPtr(), // tracked object
          &this->imuCallbackQueue // pointer to callback queue object
        );

        this->imuRosQueueThread =
          std::thread(std::bind(&NavigatePath::imuQueueThread, this));
#endif
    }
    ~NavigatePath()
    {
		dzlogfInit();
		fLog.flush();
		fLog.close();
#if TEST_SAVE_FILE
	imuLog.flush();
	imuLog.close();
#endif
	}
private:

#ifdef IMU_USE_QUEUE_THREAD
    void imuQueueThread()
    {
        static const double timeout = 0.01;
        while (mGlobal.ok())
        {
          this->imuCallbackQueue.callAvailable(ros::WallDuration(timeout));
        }
    }
#endif

    void initEstimate()
    {
#ifdef USE_MADGWICK_IMU_FILTER
        if(!mSync){
            int queueSz=1000;
            mOrentianInited=false;
            mIMUFilter.reset(new ImuFilter());
            mIMUFilter->setWorldFrame(mWorldFrame);
            mIMUFilter->setAlgorithmGain(mIMUFilterGain);
            mIMUFilter->setDriftBiasGain(mIMUFilterZeta);
            loadMagCalibraParam(mMagCalibraParam);
            mIMUSub.reset(new ImuSubscriber(mGlobal, "SensorNode/imu", queueSz));
            mMagSub.reset(new MagSubscriber(mGlobal, "SensorNode/mag",queueSz));
            mSync.reset(new Synchronizer(SyncPolicy(queueSz), *mIMUSub, *mMagSub));
            mSync->registerCallback(std::bind(&NavigatePath::imuMagCallback, this, std::placeholders::_1, std::placeholders::_2));
        }
#endif
        if (mBUseVio){
            if(!mOdomRelative){
				mSumW = 0.0;
				mTraceCounter = 0;
                mOdomRelative=mGlobal.subscribe("MotorNode/vio_odom_relative", 100, &NavigatePath::odomRelativeCB,this);
            }
        }else{
            if(!mOdomRelative){
				mSumW = 0.0;
				mTraceCounter = 0;
                mOdomRelative=mGlobal.subscribe("MotorNode/baselink_odom_relative", 100, &NavigatePath::odomRelativeCB,this);
            }
        }

#ifdef USE_SIMPLE_YAW_ESTIMATION
#ifdef NO_IMU_FILTER_NODE
        if(!mImuDataSub){

#ifndef IMU_USE_QUEUE_THREAD
            mImuDataSub=mGlobal.subscribe("/SensorNode/imu",100,&NavigatePath::imuCallback,this);
#else
            mImuDataSub = mGlobal.subscribe(imuSubOps);
#endif
        }
#else
        if(!mImuFilterTransSub){
            mImuFilterTransSub=mGlobal.subscribe("/imu/transform",100,&NavigatePath::imuFilterTransformCallback,this);
        }
#endif
#else
        if(!mImuFilterDataSub){
            mImuFilterDataSub=mGlobal.subscribe("/imu/data",100,&NavigatePath::imuFilterDataCallback,this);
        }

        if(!mImuFilterRpySub){
            mImuFilterRpySub=mGlobal.subscribe("/imu/rpy/filtered",100,&NavigatePath::imuRpyFilterCallback,this);
        }
#endif
		if(!mTof){
			mTof=mGlobal.subscribe("SensorNode/tof",1,&NavigatePath::tofDataCB,this);
		}
    }


	void calibByImuFilter()
    {
        mGotImuYawOffset = false;     ///<<< force update mImuYawOffset
        mBCalibByImuFilter = true;
#ifndef NO_IMU_FILTER_NODE
			nav_cancel ifStart;
			//call "/imu/filter_start" service for imu filter reset
			mImuFilterStartClient.call(ifStart);
			usleep(50*1000);
#else
        imuFilterStart();
#endif

#ifndef NO_IMU_FILTER_NODE
			nav_cancel ifReset;
			//call "/imu/filter_reset" service for imu filter reset
			mImuFilterResetClient.call(ifReset);
#else
        imuFilterReset();
#endif
        mImuYawOffset = 0.0;
        mGotImuYawOffset = true; ///<<<  Reset  mImuYawOffset since Imu Filter reseted
        mTimer.start();
	}

    void deinitEstimate()
    {
		mSumW = 0.0;
		mTraceCounter = 0;
		mBCalibByAkaze = false;
        mPatrolingPathName = "";
        mOdomRelative.shutdown();

#ifndef NO_IMU_FILTER_NODE
		nav_cancel ifStop;
		//call "/imu/filter_stop" service for imu filter stop
		mImuFilterStopClient.call(ifStop);
		usleep(50*1000);
#else
		imuFilterStop();
#endif

#ifdef USE_SIMPLE_YAW_ESTIMATION
#ifdef  NO_IMU_FILTER_NODE
		if(mImuDataSub) mImuDataSub.shutdown();
#else
		if(mImuFilterTransSub) mImuFilterTransSub.shutdown();
#endif
#else
		if(mImuFilterDataSub) mImuFilterDataSub.shutdown();
		if(mImuFilterRpySub) mImuFilterRpySub.shutdown();
#endif
        mTof.shutdown();
#ifdef USE_MADGWICK_IMU_FILTER
        mSync.reset();
        mMagSub.reset();
        mIMUSub.reset();
        mIMUFilter.reset();
#endif
    }
    bool start(bool reset)
    {
        mIsCancel = false;
        mDetectedPile = false;
		mDetectingPiling = false;
        stoppedAvoidObs = false;
        if(reset){
	        PLOG_INFO(NAV_PATH_NODE_TAG,"start reset");
            mCurPose=Eigen::Quaterniond(1.0,0.0,0.0,0.0);
            mCurPosition=Eigen::Vector3d(0.0,0.0,0.0);
            mCurrent.clear();
            mTriedReturnHomeOnce = false;
        }
        PLOG_DEBUG(NAV_PATH_NODE_TAG, "Start patrol");
        initEstimate();
        updateStatus();
        return mOdomRelative;
    }
    void stop()
    {
        mIsStartFromOut = false;
        mDetectedPile = false;
        if (mObjDetect){
            mObjDetect.shutdown();
            mDetectedPile = false;
        }
        stoppedAvoidObs = true;
        deinitEstimate();
        updateStatus();
    }
	bool getCalibrationStatus(nav_calibration_get_statusRequest& req,nav_calibration_get_statusResponse& res)
    {
        res.ret = mBImuPatrolCalib|mBCalibByAkaze|mBCalibByImuFilter;
        return true;
    }

    void cancelDetectCallBack(const std_msgs::Int32::ConstPtr& msg)
    {
		if (mDetectingPiling){
			mCancelDetectPile = true;
        	PLOG_INFO(NAV_PATH_NODE_TAG," cancelDetectCallBack");
		}
    }

	bool lowBatCallback(nav_low_batRequest& req,nav_low_batResponse& res)
	{
        PLOG_INFO(NAV_PATH_NODE_TAG," call lowBatCallback %f %d", mTrace.getTracePercent(), mCancelDetectPile);
        return findPileAndBackUpThread();
	}

    bool findPileAndBackUpThread(int rounds = 6)
    {
        bool bRet = false;
        mCancelDetectPile = false;
        if (!mPathPlanning &&
            !mDoBacking &&
            !mDetectingPiling &&
            !mBCalibByAkaze &&
            !mIsTracing &&
            !mBackingHome){
			if (!mDetectPileSub){
				mDetectPileSub=mGlobal.subscribe("AppNode/cancel_detect", 100, &NavigatePath::cancelDetectCallBack,this);
			}
			mDetectingPiling = true;
			auto th=std::thread([this, rounds](){
                std_msgs::Int32 goHomeStatus;
                goHomeStatus.data = status::BACK_UP_DETECT;
                mStatusPub.publish(goHomeStatus);
                if (findPile(7, M_PI/3, rounds)){
                    if(!mCharging){
                        PLOG_DEBUG(NAV_PATH_NODE_TAG, "begin backup");
                        mBackUp.begin(mGlobal);
                    } else {
                        goHomeStatus.data = status::BACK_UP_SUCCESS;
                        mStatusPub.publish(goHomeStatus);
                    }
                }else{
                    if (!mCancelDetectPile){
                        patrol_status status;
                        status.type = patrol_status::PATROL_LOSE_PILE;
                        mPatrolStatusPub.publish(status);
                    }
                    goHomeStatus.data = status::BACK_UP_INACTIVE;
                    mStatusPub.publish(goHomeStatus);
                }
                mDetectingPiling = false;
                mCancelDetectPile = false;
            });
			th.detach();
			bRet = true;
		}

		return bRet;
    }

    bool findPile(int times, float rad, int rounds=1)
    {
        PLOG_DEBUG(NAV_PATH_NODE_TAG,"findPile %d %f", times, rad);

        AlgoOBjPos pos;
        const string HOME="home";
        int timeout = 5000;
		if(mAlgo.waitObj2(HOME,timeout,pos)==0){
        	PLOG_DEBUG(NAV_PATH_NODE_TAG,"quit findPile");
			if (mCancelDetectPile){
				return false;
			}
           return true;
        }

        bool ret = false;
        timeout -= 2000;
        int moveCnt = 1;
        for (int r=0; r<rounds; r++ ){
            if(mCurBatPercentage < CRITICAL_BATTERY_PER){
                PLOG_ERROR(NAV_PATH_NODE_TAG, "Critical battery level! Stop moving around to find the charger");
                break;
            }
            for (int i=0; i<times; i++){
                if(mCharging) {
                    PLOG_DEBUG(NAV_PATH_NODE_TAG, "charged, quite filePile");
                    return true;
                }
                float w = PltConfig::getInstance()->isTrackModel() ? 3.5 : 2.5;
                int roll = mAlgo.rollAndFindPile(rad, w, DETECT_MOVE_TIMEOUT);
                if(roll == -1){
                    PLOG_DEBUG(NAV_PATH_NODE_TAG, "roll fail, try move a litlebit. moveCnt: %d", moveCnt);
                    mAlgo.move(0.0, pow(-1, moveCnt++)*DETECT_MOVEOUT_DISTANCE, DETECT_MOVE_SPEED, DETECT_MOVE_TIMEOUT);
                    continue;
                }
                else if(roll == 1) {
                    PLOG_DEBUG(NAV_PATH_NODE_TAG, "charged, quite filePile");
                    return true;
                }

                if (mCancelDetectPile){
                    return false;
                }

                if(mAlgo.waitObj2(HOME,timeout,pos)==0){
                    PLOG_DEBUG(NAV_PATH_NODE_TAG, "Found the charger!");
                    return true;
                }
            }
            if(r+1 < rounds){
                if(mCharging) {
                    PLOG_DEBUG(NAV_PATH_NODE_TAG, "charged, quite filePile");
                    return true;
                }
                float move_distance = std::min(3*DETECT_MOVEOUT_DISTANCE*(r/3 + 1), MAX_DISTANCE_TO_FIND_CHARGER);
                mAlgo.move(0.0, 3*DETECT_MOVEOUT_DISTANCE*(r/3 + 1), DETECT_MOVE_SPEED);
            }
        }
        if (mCancelDetectPile){
				return false;
		}
        return ret;
    }

	bool navExit(nav_exitRequest& req,nav_exitResponse& res)
	{
		exit(1);
		return true;
	}

	void timerCallback(const ros::TimerEvent& evt)
    {
		static int counter = 0,counter1 = 0;
		if ( 0==counter || 0==counter1 ){
			if (!mPlayingCalibAudio){
				auto th=std::thread([this](){
					mPlayingCalibAudio = true;
					playCalibAudioLoop();
				});
				th.detach();
			}
			if (!mBImuPatrolCalib && !mBCalibByImuFilter){
				return;
			}
		}

		if(true == mBImuPatrolCalib)
		{
			getimu_patrolcalib_status status;
			if (counter++<10 && mImuPatrolCalibStatusClient.call(status)){
				mBImuPatrolCalib = status.response.ret;
			}else{
				mBImuPatrolCalib = false;
			}
		}

		if(true == mBCalibByImuFilter)
		{

#ifndef NO_IMU_FILTER_NODE
			getimu_patrolcalib_status status;
			if (counter1++<10 && mImuFilterCalibStatusClient.call(status)){
				mBCalibByImuFilter = status.response.ret;
			}
#else
			if (counter1++<10 && imuFilterCalibStatus()){
				mBCalibByImuFilter = imuFilterCalibStatus();
			}
#endif
			else
			{
				mBCalibByImuFilter = false;
			}
		}

		if (!mBImuPatrolCalib && !mBCalibByAkaze && !mBCalibByImuFilter){
	        system("sudo killall aplay");
			cerr << __func__ <<  " @ line:	"<<__LINE__<< endl;
			mTimer.stop();
			cerr << endl << endl << __func__ << "Timer Stopped!!!"  << endl << endl;
			counter = 0;
			counter1 = 0;     ///<<< reset couner1s
		}
    }
    bool isDetectRecord()
    {
        detect_record_get_status req;
        if(mDetectRecordStatusClient.call(req)) {
            return req.response.status;
        }


        return false;
    }
    bool mag_calibra(nav_mag_calibraRequest& req,nav_mag_calibraResponse& res)
    {
        if(mDoingMagCalibra){
            return false;
        }
        mDoingMagCalibra=true;
        auto th=std::thread([this](){
            mAlgo.magneticCalibrate();
            mDoingMagCalibra=false;
        });
        th.detach();
        return true;
    }

	void start_path_out(string pathName)
	{
		mBImuPatrolCalib = true;
		saveTmpPicForStartPath savePath;
		savePath.request.name = NAVIGATE_JPG_ROOT_PATH+pathName+".jpg";
		mSaveTmpPicClient.call(savePath);
		geometry_msgs::Twist twist;
        twist.linear.x=0;
        twist.linear.y=0.1;
        twist.angular.z=0;
        mCmdVel.publish(twist);
        usleep(500*1000);
		twist.linear.y=0.0;
		mCmdVel.publish(twist);
        usleep(500*1000);
		savePath.request.name = NAVIGATE_JPG_ROOT_PATH+pathName+"_1.jpg";
		mSaveTmpPicClient.call(savePath);

		imu_patrol_calib ipcalib;
		mImuPatrolCalibClient.call(ipcalib);
		mTimer.start();
	}
    bool start_path(nav_path_startRequest& req,nav_path_startResponse& res)
    {
        if(mIsTracing){
            return false;
        }
        mDoBacking=false;
        mPathPlanning = true;
        mPatrolingPathName = req.name;
        mIsStartFromOut = req.isFromOutStart;

		if (!mPatrolingPathName.empty()){
			start_path_out(mPatrolingPathName);
		}else{
			geometry_msgs::Twist twist;
			twist.linear.x=0;
			twist.linear.y=0.1;
			twist.angular.z=0;
			mCmdVel.publish(twist);
		}
		// auto th=std::thread([this](mPatrolingPathName){
        //             start_path_calib(mPatrolingPathName);
        //         });
        // th.detach();
		usleep(500*1000);
		calibByImuFilter();
        return start(true);
    }

    bool cancel(nav_cancelRequest& req,nav_cancelResponse& res)
    {
        mAlgo.quitAlgo();
        mPathPlanning = false;
        stop();

        mDoBacking=false;
        setPatrolStatus(false);
        mIsCancel = true;
#ifndef NO_IMU_FILTER_NODE
		nav_cancel ifStop;
		mImuFilterStopClient.call(ifStop);
		usleep(50*1000);
#else
		imuFilterStop();
#endif
		return true;
    }

    bool getName(nav_get_patrol_name::Request& request,
                                  nav_get_patrol_name::Response& response)
    {
        if(!mIsTracing){
            response.name = "";
            return false;
        }

        response.name = mPatrolingPathName;
        return true;
    }
    bool getStatus(nav_get_status::Request& request,
                                  nav_get_status::Response& response)
    {
        response.status = mIsTracing;

        return true;
    }


    bool addWayPt(nav_waypoint_add::Request& request,
                                  nav_waypoint_add::Response& response)
    {
        if (!mPathPlanning){
            return false;
        }

        return true;
    }


    bool queryPt(nav_waypoint_query::Request& request,
                                  nav_waypoint_query::Response& response)
    {
        return true;
    }

    bool enableVio(enable_vio::Request& request,
                                  enable_vio::Response& response)
    {
        mBUseVio = request.enable;
        return true;
    }


    bool save_path(nav_path_saveRequest& req,nav_path_saveResponse& res)
    {
        if(mIsTracing){
            return false;
        }
        if(mCurrent.size()==0){
            return false;
        }
        bool ret=true;
        if(req.name.size()>0){//if req.name is empty string,only stop odom not save
            {
                ret = save_path(req.name);
                std_msgs::Int32 status;
                status.data = 1;
                status.data = ret?0:1;
                mNavTraceDonePub.publish(status);
                return ret;
            }
        }
        stop();
        return ret;
    }

    bool save_path(string name)
    {
        bool isDoBacking = mDoBacking;
        bool isStartFromOut = mIsStartFromOut;
        bool ret=true;
        string path=NAVIGATE_PATH_PATH+name;
        ret=(mCurrent.saveToFile(path)==0);
        if(!ret){
        }else{
            PltConfig::getInstance()->updateLastPatrolName(name);
			string oname = NAVIGATE_JPG_ROOT_PATH+name+".jpg_tmp";
			string newName = NAVIGATE_JPG_ROOT_PATH+name+".jpg";
    		rename(oname.c_str(),newName.c_str());
			oname = NAVIGATE_JPG_ROOT_PATH+name+"_1.jpg_tmp";
			newName = NAVIGATE_JPG_ROOT_PATH+name+"_1.jpg";
    		rename(oname.c_str(),newName.c_str());
        }
        stop();

        if (isStartFromOut && isDoBacking){
            auto th=std::thread([this](){
                    mBackUp.begin(mGlobal);
                });
            th.detach();
        }
        return ret;
    }

    void updateStatus()
    {
        int status[2];
        status[0]=mOdomRelative?1:0;
        status[1]=mIsTracing?1:0;
        mPub.pubStatus(&status[0],sizeof(status)/sizeof(status[0]));
    }
    void setPatrolStatus(bool flag)
    {
        std_msgs::Int32 event;
        if (!mIsTracing && !mDoBacking && flag){
            mIsPatroling = true;
            patrol_status status;
            status.type = patrol_status::START_PATROL;
            status.name = mPatrolingPathName;
            mPatrolStatusPub.publish(status);
        }else if (mIsPatroling && !flag){
            mIsPatroling = false;
            patrol_status status;
            status.type = patrol_status::END_PATROL;
            status.name = mPatrolingPathName;
            mPatrolStatusPub.publish(status);
        }

        mIsTracing=flag;
        updateStatus();
    }
    bool stopPatrol()
    {
        printf("mObstracks->size(%d) !!!!!!!!\r\n",mObstracks->size());

        /*if(!mDoBacking)*/{
            stop();
        }
        mDoBacking=false;
        setPatrolStatus(false);

        return true;
    }

    bool isMinDist2Front()
    {
        if (mCurrent.size()==0){
            return true;
        }

         TrackTrace trace;
        auto track=make_shared<TrackList>();
        *track=mCurrent;
        trace.setTrackList(track);
        double dist = trace.distanceFrontPoint(mCurPose, mCurPosition);
        return dist<NAV_MANUALSAVE_MIN_DIST;
    }

	bool rollForAkaze(float angle)
	{
		float minAngle = M_PI/60;
 		if(abs(angle)>minAngle/2 && abs(angle)<minAngle){
			if (angle>0){
				angle += minAngle/2;
			}else{
				angle -= minAngle/2;
			}
		}

		if (abs(angle)<=minAngle){
			return false;
		}

		mAlgo.roll(angle,TRACK_ROLL_SPEED);
		return true;
	}

/*************************************************
Function:       calibByAkaze
Description:   According to the pictures saved when planning the path,
                            calibrate the pile out angle of Scout before starting patrol.
							Make a calibration on the charging pile and a calibration after leaving the charging pile.
Input:         pathName-Name of patrol path
Output:
Return:
*************************************************/
	void calibByAkaze(string pathName)
	{
		//Start the timer and start playing the calibration sound in timerCallback
		mTimer.start();

		getDiffAngleWhenPatrol diff;

	     //Pile out
		geometry_msgs::Twist twist;
		twist.linear.x=0;
		twist.linear.y=0.1;
		twist.angular.z=0;
		mCmdVel.publish(twist);
		usleep(500*1000);
		twist.linear.y=0.0;
		mCmdVel.publish(twist);
		usleep(500*1000);

		mBImuPatrolCalib = true;
		imu_patrol_calib ipcalib;
		double time1 = ros::Time::now().toSec();
		//call "/imu_patrol_calib" service for imu calibration
		mImuPatrolCalibClient.call(ipcalib);

        PLOG_DEBUG(NAV_PATH_NODE_TAG, "Waiting for calib IMU");

		while(mBImuPatrolCalib)
		{
			usleep(100*1000);
			double time2 = ros::Time::now().toSec();
			// PLOG_DEBUG(NAV_PATH_NODE_TAG, "(Time2，Time1= ，%f , %f )",time2,time1);
			if ((time2 -time1) > CALIB_MAX_WAIT_TIME)
			{
				mBImuPatrolCalib = false;   ///timeout occus
				break;
			}
		}
		calibByImuFilter();

        PLOG_INFO(NAV_PATH_NODE_TAG, "Doing the last Akaze");

        diff.request.name = NAVIGATE_JPG_ROOT_PATH+pathName+"_1.jpg";
        if (mGetDiffAngleClient.call(diff)){
            m_Yaw_Save = -diff.response.angle;
            PLOG_INFO(NAV_PATH_NODE_TAG,"calibByAkaze3: %f, -> set initial angle: %f",diff.response.angle, m_Yaw_Save);
        }
		mBCalibByAkaze = false;

	}

    bool patrol(nav_patrolRequest& req,nav_patrolResponse& res)
    {
		res.ret = 0;
        if(mIsTracing){
            return false;
        }
        mIsCancel = false;
        mDoBacking=false;
        mIsStartFromOut = false;
        auto track=make_shared<TrackList>();
        if(req.name==""){//back to home
            *track=mCurrent;
            mDoBacking=true;
            if (!mDetectingPiling &&
                 !mPatrolingPathName.empty()/* &&
                 isMinDist2Front()*/){
                 mDetectingPiling = true;
                 auto th=std::thread([this](){
                            courseReversal();
                        });
                th.detach();
                return true;
            }
			mTrace.setTrackList(track);
        }else{
            string path = NAVIGATE_PATH_PATH + req.name;
            mPatrolingPathName = req.name;
            mIsStartFromOut = req.isFromOutStart;

            if(track->loadFromFile(path)<0){
                return false;
            }else{
                PltConfig::getInstance()->updateLastPatrolName(req.name);
            }
            track->erase(0.3);
			mTrace.setTrackList(track);
			if (mCurBatPercentage<mMinBat){
				res.ret = 2;
                return true;
            }
			mBCalibByAkaze = true;
			auto th=std::thread([this](){
                    calibByAkaze(mPatrolingPathName);
                });
            th.detach();
            PLOG_DEBUG(NAV_PATH_NODE_TAG,"path size=%d",track->size());
        }
		//mTrace.setTrackList(track);

        cerr << __func__ <<  " # line:  "<<__LINE__<< "@time:  "<<ros::Time::now()<< endl;
		mTimer.start();
		ros::Time T0=ros::Time::now(),T1;
		ros::TimerEvent tEvt;

		if(0)
			while(mBImuPatrolCalib||mBCalibByAkaze||mBCalibByImuFilter)
		{
			cerr << __func__ <<  " @ line:	"<<__LINE__<< "@time:  "<<ros::Time::now()<< endl;
			usleep(100*1000);
			T1= ros::Time::now();
			if(fmod((T1-T0 ).toSec() ,2.10)>= 1.90)
			{
			///	T0 = T1;
				if(mTimer)
				{
					cerr << "mTimer.isValid(): " << mTimer.isValid() << "   ";
					cerr << "mTimer.hasStarted(): " << mTimer.hasStarted() << "   ";
					cerr << "mTimer.hasPending(): " << mTimer.hasPending() << "   ";
					cerr << endl << endl;
				}
				///else
				if((T1-T0 ).toSec() >= 9.90)
				{
					cerr << endl<<endl<<"Force call timerCallback()!!!" <<endl<<endl;
					timerCallback(tEvt);
					T0 = T1 ;
				}
			}
			cerr << "(mBImuPatrolCalib||mBCalibByAkaze||mBCalibByImuFilter)= " ;
			cerr << mBImuPatrolCalib << mBCalibByAkaze << mBCalibByImuFilter << endl;
		}

		if(!start(!mDoBacking)){
			track->clear();
            return false;
        }

        mObstracks = track;
        setPatrolStatus(true);
        return true;
    }
    bool stop_patrol(nav_patrol_stopRequest& req,nav_patrol_stopResponse& res)
    {
        return stopPatrol();
    }
    bool list_path(nav_list_pathRequest& req,nav_list_pathResponse& res)
    {
        string path=NAVIGATE_PATH_PATH;
        vector<string> vFiles;
        bool bRet = listDir(path, vFiles);
        for (auto filename : vFiles){
            string fullPathName = path + filename;
            struct stat s;
            if (stat(fullPathName.c_str(), &s) == 0){
                    char buff[128];
                    time_t tTimeTmp = 0;
                    struct tm stuTimeTmp;
                    memset(&stuTimeTmp,0,sizeof(struct tm));
                    tTimeTmp = s.st_ctime;
                    gmtime_r(&tTimeTmp,&stuTimeTmp);

                    sprintf(buff, "%d-%02d-%02d  %02d:%02d:%02d",
                                    stuTimeTmp.tm_year + 1900,
                                    stuTimeTmp.tm_mon + 1,
                                    stuTimeTmp.tm_mday,
                                    stuTimeTmp.tm_hour,
                                    stuTimeTmp.tm_min,
                                    stuTimeTmp.tm_sec );

                    res.size_list.push_back(s.st_size);
                    res.name_list.push_back(filename);
                    res.path_list.push_back(fullPathName);
                    res.create_time_list.push_back(buff);
            }
        }

        return bRet;

    }
    bool delete_path(nav_delete_pathRequest& req,nav_delete_pathResponse& res)
    {
        bool ret=true;
        for(auto& name:req.names){
            string path=NAVIGATE_PATH_PATH+name;
            if(remove(path.c_str())!=0){
                ret=false;
            }
			path = NAVIGATE_JPG_ROOT_PATH+name+".jpg";
            if(remove(path.c_str())!=0){
                ret=false;
            }
			path = NAVIGATE_JPG_ROOT_PATH+name+"_1.jpg";
            if(remove(path.c_str())!=0){
                ret=false;
            }
        }
        return ret;
    }

	static void toEulerAngle(const Eigen::Quaterniond& q, double& roll, double& pitch, double& yaw)
	{
		// roll (x-axis rotation)
		double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
		double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
		roll = atan2(sinr_cosp, cosr_cosp);

		// pitch (y-axis rotation)
		double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
		if (fabs(sinp) >= 1)
		pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
		else
		pitch = asin(sinp);

		// yaw (z-axis rotation)
		double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
		double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
		yaw = atan2(siny_cosp, cosy_cosp);
	}

    void odomRelativeCB(const nav_msgs::Odometry::ConstPtr& msg)
    {
		if (mBImuPatrolCalib || mBCalibByAkaze || mBCalibByImuFilter ){
			return;
		}

        Eigen::Vector3d t(msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);
        float vx=msg->twist.twist.linear.x;
        float vy=msg->twist.twist.linear.y;
        float w=msg->twist.twist.angular.z;

        if (mBUseVio){
            Eigen::Quaterniond cp(msg->pose.pose.orientation.w,msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z);
            mCurPosition = t;
            mCurPose       = cp;
        }else{
            #ifndef USE_MADGWICK_IMU_FILTER

			#ifndef D_USE_IMU_FILTER_POSE
			mCurPose=mCurPose*Eigen::Quaterniond(msg->pose.pose.orientation.w,msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z);
			#endif
			mCurPosition += mCurPose*t;
            #else
            if(!mOrentianInited){
                return;
            }
            mCurPosition+=mCurPose*t;
            mCurPosition.z()=0.0;
            mIMUFilter->getOrientation(mCurPose.w(),mCurPose.x(),mCurPose.y(),mCurPose.z());
            #endif
        }


	 	 mSumW += msg->pose.pose.orientation.z;

		double yaw = 360*mSumW/M_PI;
		int nsumw = int(100*yaw)%36000;
		yaw  = float(nsumw)/100.0;

        geometry_msgs::PoseStamped pose;
        pose.header.frame_id="world";
        pose.header.stamp=msg->header.stamp;

        pose.pose.position.x=mCurPosition.x();
        pose.pose.position.y=mCurPosition.y();
        pose.pose.position.z=mCurPosition.z();
        pose.pose.orientation.w=mCurPose.w();
        pose.pose.orientation.x=mCurPose.x();
        pose.pose.orientation.y=mCurPose.y();
	#ifndef D_USE_IMU_FILTER_POSE
		pose.pose.orientation.z=yaw;
	#else
		pose.pose.orientation.z=mImuFilterYaw;      ///<<< use Yaw from IMU filter output...
	#endif
		mPose.publish(pose);

		double roll,pitch,yaw1;
		toEulerAngle(mCurPose,roll, pitch,yaw1);

	    if (mDetectingPiling){
            return;
        }

        if(mCurrent.size()>mMaxPathSize){
            stopPatrol();
            stop();
        }else{
            if(!mAdjusting){
                TrackPoint point(msg->header.stamp.toNSec(),mCurPosition,mCurPose,vx,vy,w);
                mCurrent.push(point);
            }
            doTrace(msg->header.stamp.toSec());
        }
    }
    void tofDataCB(const sensor_msgs::RangeConstPtr &r)
    {
        if(r->range>=0 && r->range<NAV_OBSTACLE_MIN_DIST){
            mDetectObstacleCnt++;
        }else{
            mDetectObstacleCnt=0;
        }

        float range = 0;

        if(isinf(r->range)){
            range = 2.0;
        }else{
            range = r->range;
        }
        //mTofDistance = range;
        mAvgTof = mAvgTof+(range-mAvgTof)/mTofAvgCnt;
        // mAvgTof = range;
        mTofDistance=mAvgTof;
    }

    void doTrace(double stamp)
    {
        if(!mIsTracing && !mDoBacking){
            return;
        }
	    boost::mutex::scoped_lock lock(mutex_trace);	 /// Add mutex lock for tracing;

        //PLOG_DEBUG(NAV_PATH_NODE_TAG,"doTrace %d %d", mIsPatroling,mTrace.size());
        if(mTrace.done() || mDetectedPile){
            traceDone(); //rmwei
            return;
        }
        if (mIsPatroling && isDetectRecord()){
            return;
        }

        mLastTraceStamp=stamp;

        if(mAdjusting){
            return;
        }

        if(mDetectObstacleCnt > NAV_OBSTACLE_CNT){
            std_msgs::Int32 event;
            event.data = SYSEVT_OBSTACLE;
            mSysEvtPub.publish(event);
            startAvoidObstacle();
            return;
        }

        traceOnce();
    }

    void traceDone()
    {
        if (!mIsPatroling && !mPatrolingPathName.empty()){
            std_msgs::Int32 status;
            status.data = 1;
            bool bRet = save_path(mPatrolingPathName);
            status.data = bRet?0:1;
            mNavTraceDonePub.publish(status);
        }

        mPathPlanning = false;
        if (!mIsPatroling ||
            (mIsPatroling && mIsStartFromOut)){
                                            //   mIsPatroling, mIsStartFromOut,  __FILE__, __LINE__);
            stopPatrol();

            PLOG_ERROR(NAV_PATH_NODE_TAG,"traceDone...");
            auto th=std::thread([this](){
                    mBackUp.begin(mGlobal);
                });
            th.detach();
        }else{
            stopPatrol();
        }
    }
    void onObjdetect(const detectConstPtr& obj)
    {
        PLOG_DEBUG(NAV_PATH_NODE_TAG, "Detected the Pile!");
        mDetectedPile = true;
    }

    void traceOnce()
    {
        mTrace.updatePose(mCurPosition,mCurPose);
        if(mClipData){
            mTrace.clipTrace(50);
            mClipData=false;
        }
        if (mTrace.size()<=20 &&
             mIsStartFromOut &&
             !mObjDetect){   //40cm
             mObjDetect=mGlobal.subscribe("CoreNode/chargingPile",1,&NavigatePath::onObjdetect,this);
        }

		mTraceCounter++;
        float vx,vy,w,roll;
        if(mTrace.traceOnce(vx,vy,w,roll)){
            PLOG_DEBUG(NAV_PATH_NODE_TAG,"doTrace  startRoll:%f",roll);
            startRoll(roll);
        }else{
            geometry_msgs::Twist twist;
            twist.linear.x=vx;
            twist.linear.y=vy;
			if (mTraceCounter<20){
           		twist.linear.y=vy/2;
			}else if (mTraceCounter<40){
           		twist.linear.y=mTraceCounter*vy/40;
			}
            twist.angular.z=w;
            mCmdVel.publish(twist);
        }
    }

    void startRoll(double angle)
    {
        mAdjusting=true;
        auto th=std::thread([this,angle](){
            mAlgo.roll(angle,TRACK_ROLL_SPEED);
            mAdjusting=false;
        });
        th.detach();
    }

    void startAvoidObstacle()
    {
        if (mAdjusting){
            return;
        }

		mAlgo.action(0.0, 0.0, 0.0, 600); /// stop the scout first, RogerL 22/11/23

        mAdjusting=true;
        isObs = true;
        stoppedAvoidObs = false;

        auto th=std::thread([this](){
            boost::mutex::scoped_lock lock(mutex_OBS);   /// Add mutex lock for avoid obstacle;
        	PLOG_INFO(NAV_PATH_NODE_TAG,"avoidObstacle thread start.");
            if(avoidObstacleH()) {
                isObs = false;
            }
            else {
                mTrace.clear();
            }

            if(mDetectedPile) {
                PLOG_INFO(NAV_PATH_NODE_TAG, "Detected the charger, going back to the charger.")
                ObsBackHomeThread();
            }
            mAdjusting=false;
        });
        th.detach();
    }

	typedef enum {
		E_OBS_DIRLEFT		= 1,
		E_OBS_DIRRIGHT,
		E_OBS_RETURN,
		E_OBS_BACKHOME,
		E_OBS_BACKUP
	} OBS_STATUS;

    void calDistanceAndAngle(Eigen::Quaterniond& q,Eigen::Vector3d& pos,Eigen::Vector3d &goal,double &angle,double &distance)
    {
#ifdef 	AVOID_OBSTACLE_WITH_MUTEX	///DEBUG_AVOID_OBSTACLE
        boost::mutex::scoped_lock lock(mutex_calcDistAngle);   /// Add mutex lock for calc distance & angle;
#endif
        Eigen::Vector3d d=q.inverse()*(goal-pos);
        distance=d.norm();
        if(distance<0.1/2){
            angle=0;
            return;
        }
        angle=std::asin(d.y()/distance);
        if(d.x()<0){
            angle=(angle>0?(M_PI-angle):(-M_PI-angle));
        }
        angle-=M_PI_2;
        if(angle<-M_PI){
            angle+=2*M_PI;
        }
    }

	void playCalibAudioLoop()
	{
	    json param;
        PltConfig *config = PltConfig::getInstance();
        config->getSoundEffectParam(param);
        int activate = param["activate"];
        if (!activate){
            return;
        }
		while(mBImuPatrolCalib || mBCalibByAkaze || mBCalibByImuFilter){
        	string cmd = "aplay /var/roller_eye/devAudio/calib.wav";
        	system(cmd.c_str());
			// printf("%s\r\n",cmd.c_str());
		}
		mPlayingCalibAudio = false;
	}

#define OBS_FURTHER_DIST_MAX_CNT        2
#define OBS_TARGET_DISTANCE             0.6f
#define OBS_TARGET_DISTANCE_MAX         1.5f
#define NAV_OBS_FREE_SPACE_DIST         0.015f
	void ObsBackHomeThread(){
		mDoBacking = true;
		//mDetectingPiling = true;
		PLOG_INFO(NAV_PATH_NODE_TAG,"===>mBackUp.begin.\r\n");
		setPatrolStatus(false);
		stop();
		auto th=std::thread([this](){
			mBackUp.begin(mGlobal);
		});
		th.detach();
	}

    bool scanObsProfile(int &direction, int cnt){
        double obsAngle, obsDist;
        if (mDetectedPile || stoppedAvoidObs) return false;
        if(direction == -1) {
            auto pdictLeft = make_shared<DistanceMap>();
            double obsAngleLeft = -1;
            double obsAngleRight = -1;
            PLOG_DEBUG(NAV_PATH_NODE_TAG, "Scan OBS");
            if (mDetectedPile || stoppedAvoidObs) return false;
            if(mAlgo.getDistance(M_PI, pdictLeft) == 1) {
                obsAngleLeft = abs(pdictLeft->back().first);
            }
            PLOG_DEBUG(NAV_PATH_NODE_TAG, "obs angle left: %f", obsAngleLeft);

            if (mDetectedPile || stoppedAvoidObs) return false;
            mAlgo.rollEx(- pdictLeft->back().first, mObsWz, DURATION_10_S, DGREE_2_RAD);

            usleep(1000*1000);
            auto pdictRight = make_shared<DistanceMap>();
            if (mDetectedPile || stoppedAvoidObs) return false;
            if(mAlgo.getDistance(-M_PI, pdictRight) == 1) {
                obsAngleRight = abs(pdictRight->back().first);
            }
            PLOG_DEBUG(NAV_PATH_NODE_TAG, "obs angle right: %f", obsAngleRight);
            usleep(1000*1000);

            if (obsAngleLeft < 0 && obsAngleRight < 0){
                return false;
            }

            if (obsAngleLeft < obsAngleRight) {
                direction = 0; // obs left side
                auto pdictLeft = make_shared<DistanceMap>();
                if (mDetectedPile || stoppedAvoidObs) return false;
                mAlgo.rollEx(obsAngleRight, mObsWz);
                usleep(500*1000);
                if (mDetectedPile || stoppedAvoidObs) return false;
                mAlgo.getDistance(obsAngleLeft + M_PI_2, pdictLeft);
                obsDist = (pdictLeft->end() - 2)->second;
                usleep(500*1000);
                mAlgo.rollEx(atan2(NAV_OBS_FREE_SPACE_DIST, obsDist), mObsWz);
                PLOG_DEBUG(NAV_PATH_NODE_TAG, "Go around the left side cnt: %d, ang: %f, dist: %f", cnt, obsAngleRight, obsDist);
                return true;
            }
            else {
                direction = 1; // obs right side
                obsDist = (pdictRight->end() - 2)->second;
                mAlgo.rollEx(-atan2(NAV_OBS_FREE_SPACE_DIST, obsDist), mObsWz);
                PLOG_DEBUG(NAV_PATH_NODE_TAG, "Go around the right side cnt: %d ang: %f, dist: %f", cnt, obsAngleRight, obsDist);
                return true;
            }
        }
        else if (direction == 0){ // go around on the left side
            auto pdictLeft = make_shared<DistanceMap>();
            // PLOG_DEBUG(NAV_PATH_NODE_TAG, "Scan obs on the left");
            if (mDetectedPile || stoppedAvoidObs) return false;
            if (mAlgo.getDistance(M_PI * 1.5, pdictLeft) != 1) return false;
            obsAngle = pdictLeft->back().first;
            obsDist = (pdictLeft->end() - 2)->second;
            usleep(500*1000);
            mAlgo.rollEx(atan2(NAV_OBS_FREE_SPACE_DIST, obsDist), mObsWz);
            PLOG_DEBUG(NAV_PATH_NODE_TAG, "Go around the left side cnt: %d: ang: %f, dist: %f", cnt, obsAngle, obsDist);
            return true;
        }
        else if (direction == 1){ // go around on the Right side
            auto pdictRight = make_shared<DistanceMap>();
            // PLOG_DEBUG(NAV_PATH_NODE_TAG, "Scan obs on the right");
            if (mDetectedPile || stoppedAvoidObs) return false;
            if (mAlgo.getDistance(-M_PI * 1.5, pdictRight) != 1) return false;
            obsAngle = pdictRight->back().first;
            obsDist = (pdictRight->end() - 2)->second;
            usleep(500*1000);
            mAlgo.rollEx(-atan2(NAV_OBS_FREE_SPACE_DIST, obsDist), mObsWz);
            PLOG_DEBUG(NAV_PATH_NODE_TAG, "Go to the right side cnt: %d ang: %f, dist: %f", cnt, obsAngle, obsDist);
            return true;
        }
    }

    bool avoidObstacleH()
    {
        bool ret = false;
        mCurrent.clip(mCurPosition, 5);
        int start_obs_mCurrent_size = mCurrent.size();
        Eigen::Vector3d start_obs_position = mCurrent.TrackPointList().rbegin()->position;

        PLOG_DEBUG(NAV_PATH_NODE_TAG,"start avoid obstacle #%d loop, at point %d in mCurrent", ++mObsCnt, start_obs_mCurrent_size);
        shared_ptr<TrackList> obsTracksList = mTrace.getTrackList();
        PLOG_DEBUG(NAV_PATH_NODE_TAG, "Track list size 0: %d", mTrace.size());
        pair<pair<double, double>, int> target_point;
        pair<double, double> *old_target_point = nullptr;
        int avoid_dir = -1; // -1: not set
                            //  0: Left
                            //  1: Right
        int cnt = 0;
        double lastDistance = 0.0;
        double tofDistance = -1.0;
        int cntGoFuther = 0;
        bool triedOtherSideOnce = false;

        if (obsTracksList->size() == 0) {
            return true;
        }

        while(cnt < NAV_OBSTACLE_CNT) {
            if(mDetectedPile || stoppedAvoidObs) return false;
            ///< 1. If too close obs -> move back
            usleep(500*1000);
            tofDistance = mTofDistance;
            if (tofDistance < OBS_TOO_CLOSE_DIST) {
                mAlgo.move(0.0, -OBS_MOVE_BACK_DIST, OBS_MOVE_SPEED);
                usleep(500*1000);
                tofDistance = mTofDistance;
            }
            PLOG_DEBUG(NAV_PATH_NODE_TAG, "Distance to obs #%d: %f", tofDistance);

            ///< 2. Find target_point
            ///<    1. Target point is the first point in trackList
            //      2.
            int index = 0;
            int count = 0;
            // Find target_point
            PLOG_DEBUG(NAV_PATH_NODE_TAG, "Track list size 1: %d", obsTracksList->size());
            for (auto it = obsTracksList->TrackPointList().rbegin();
                    it != obsTracksList->TrackPointList().rend(); it++){
                double tangle = 0;
                double tdistance = 0;
                calDistanceAndAngle(mCurPose, mCurPosition, it->position, tangle, tdistance);
                if (tdistance >= lastDistance) {
                    pair<double, double> tmp(tdistance, tangle);
                    target_point.first = tmp;
                    target_point.second = index;
                    lastDistance = tdistance;
                } else {
                    if (count++ > 10) {
                        break;
                    }
                }
                if (index++ > 300){
                    break;
                }
                if (tdistance > OBS_TARGET_DISTANCE) {
                    cnt++;
                    break;
                }
            }
            PLOG_DEBUG(NAV_PATH_NODE_TAG, "OBS target_point: (%f, %f), %d", target_point.first.first, target_point.first.second, target_point.second);

            if(target_point.second > 0){
                PLOG_DEBUG(NAV_PATH_NODE_TAG, "Clip %d point in the track list", target_point.second);
                mTrace.clipTrace(target_point.second);
                PLOG_DEBUG(NAV_PATH_NODE_TAG, "Track list size 2: %d", obsTracksList->size());
            }

            ///< Check near the end
            if(target_point.first.first < OBS_TARGET_DISTANCE && obsTracksList->size() == 1) {
                PLOG_INFO(NAV_PATH_NODE_TAG, "Near the end!");
                return false;
            }

            if(obsTracksList->size() <= 20 && !mObjDetect) {
                PLOG_INFO(NAV_PATH_NODE_TAG, "Near the end, find the chargerPile");
                mDetectedPile = false;
                mObjDetect = mGlobal.subscribe("CoreNode/chargingPile", 1, &NavigatePath::onObjdetect, this);
            }

            if(mDetectedPile || stoppedAvoidObs) return false;
            ///< Rotate to the target_point
            PLOG_DEBUG(NAV_PATH_NODE_TAG, "Angle to target: %f, free angle: %f", target_point.first.second, atan2(0.03, tofDistance));
            if(abs(target_point.first.second) > atan2(0.03, tofDistance)){
                PLOG_DEBUG(NAV_PATH_NODE_TAG, "Rotate to target: %f", target_point.first.second);
                mAlgo.rollEx(target_point.first.second, mObsWz);
            }
            usleep(100*1000);

            // Just for DEBUG, check if the above rotate to true direction
            double tAngle1, tDistance1;
            calDistanceAndAngle(mCurPose, mCurPosition, obsTracksList->TrackPointList().rbegin()->position, tAngle1, tDistance1);
            PLOG_DEBUG(NAV_PATH_NODE_TAG, "angle to target_point after roll to the target: %f", tAngle1);

            tofDistance = mTofDistance;
            // Check is there any obs in the direction to target_point
            if (tofDistance > target_point.first.first) {
                PLOG_INFO(NAV_PATH_NODE_TAG, "Pass the OBS #%d", mObsCnt);
                return true;
            }

            Eigen::Vector3d target_point_pos = obsTracksList->TrackPointList().rbegin()->position;

            ///< Scan to find angle
            if((cntGoFuther > OBS_FURTHER_DIST_MAX_CNT
                || tDistance1 > OBS_TARGET_DISTANCE_MAX) && !triedOtherSideOnce) {
                PLOG_INFO(NAV_PATH_NODE_TAG, "Try to other side");
                avoid_dir = 1 - avoid_dir; // Try to other side
                cnt = 0; // reset cnt
                cntGoFuther = -10; // Do not change side again
                triedOtherSideOnce = true;
                // Move back to the start avoidance position
                double tAng, tDis;
                calDistanceAndAngle(mCurPose, mCurPosition, start_obs_position, tAng, tDis);
                mAlgo.rollEx(tAng, mObsWz);
                usleep(1000*1000);
                PLOG_DEBUG(NAV_PATH_NODE_TAG, "Check whether go directly to starting point: tDis: %f, mTof: %f", tDis, mTofDistance);
                if(mTofDistance > tDis){
                    mAlgo.move(0.0, tDis, OBS_MOVE_SPEED);
                } else {
                    mAlgo.rollEx(-tAng, mObsWz);
                    cnt = -2;
                }
            }
            if (!scanObsProfile(avoid_dir, cnt)) {
                return false;
            }

            if(mDetectedPile || stoppedAvoidObs) return false;
            mAlgo.move(0.0, OBS_MOVE_FORWARD_DIST, OBS_MOVE_SPEED);

            double tAngle2, tDistance2;
            calDistanceAndAngle(mCurPose, mCurPosition, obsTracksList->TrackPointList().rbegin()->position, tAngle2, tDistance2);
            if(mDetectedPile || stoppedAvoidObs) return false;
            mAlgo.rollEx(tAngle2, mObsWz);
            usleep(1000*1000);
            if(tDistance2 > tDistance1) {
                cntGoFuther++;
            }
            tofDistance = mTofDistance;
            if (tofDistance > tDistance2) {
                PLOG_INFO(NAV_PATH_NODE_TAG, "Pass the OBS #%d", mObsCnt);
                return true;
            }
            lastDistance = tDistance2;
        }

        PLOG_ERROR(NAV_PATH_NODE_TAG, "Cannot pass the OBS, go back home");
        if(mTriedReturnHomeOnce) {
            PLOG_ERROR(NAV_PATH_NODE_TAG, "OBS avoidance failed!!!!");
            patrol_status status;
            status.type = patrol_status::PATROL_AVOID_OBS_FAIL;
            mPatrolStatusPub.publish(status);
            return false;
        }
        mDoBacking = true;
        mTriedReturnHomeOnce = true;
        avoid_dir = 1 - avoid_dir;
        // Back to the path
        int back_cnt = 0;
        while (++back_cnt < NAV_OBSTACLE_CNT)
        {
            double tAng, tDis;
            calDistanceAndAngle(mCurPose, mCurPosition, start_obs_position, tAng, tDis);
            if(mDetectedPile || stoppedAvoidObs) return false;
            mAlgo.rollEx(tAng, mObsWz);
            usleep(1000*1000);
            if(mTofDistance > tDis){
                if(mDetectedPile || stoppedAvoidObs) return false;
                mAlgo.move(0.0, tDis, OBS_MOVE_SPEED);
                break;
            }
            else {
                if(mDetectedPile || stoppedAvoidObs) return false;
                if (!scanObsProfile(avoid_dir, back_cnt));
                mAlgo.move(0.0, OBS_MOVE_FORWARD_DIST, OBS_MOVE_SPEED);
            }
        }

        if (!mDetectingPiling) {
            mDetectingPiling = true;
            if(!mObjDetect){
                mObjDetect = mGlobal.subscribe("CoreNode/chargingPile", 1, &NavigatePath::onObjdetect, this);
            }
            auto th = std::thread([this]() {
                PLOG_DEBUG(NAV_PATH_NODE_TAG, "mCurrent size: %d", mCurrent.size());
                courseReversal(false);
            });
            th.detach();
        }
    }

    bool detectPileInSitu(int times, float rad)
    {
        PLOG_DEBUG(NAV_PATH_NODE_TAG,"detectPileInSitu %d %f", times, rad);

        AlgoOBjPos pos;
        const string HOME="home";
        int timeout = 5000;
		if(mAlgo.waitObj2(HOME,timeout,pos)==0){
        	PLOG_DEBUG(NAV_PATH_NODE_TAG,"quit detectPileInSitu");
			if (mCancelDetectPile){
				return false;
			}
           return true;
        }

        bool ret = false;
        timeout -= 2000;
        for (int i=0; i<times; i++){
            if(mAlgo.roll(rad,1.0)<0){
        			PLOG_DEBUG(NAV_PATH_NODE_TAG,"quit detectPileInSitu %d", i);
                    break;
            }

			if (mCancelDetectPile){
				return false;
			}

  			if(mAlgo.waitObj2(HOME,timeout,pos)==0){
                    ret = true;
                    break;
            }
        }
        if (mCancelDetectPile){
				return false;
		}
        return ret;
    }

    void courseReversal(bool savePath=true)
    {
        if (detectPileInSitu(1, M_PI)
		     && mDoBacking){
            std_msgs::Int32 status;
            status.data = 1;
            if(savePath) {
                bool bRet = save_path(mPatrolingPathName);
                status.data = bRet?0:1;
                mNavTraceDonePub.publish(status);
            }
            auto th=std::thread([this](){
                    mBackUp.begin(mGlobal);
                });
            th.detach();
        }else{
            if (mDoBacking){
                auto track=make_shared<TrackList>();
                *track=mCurrent;
                mTrace.setTrackList(track);
                if(!start(!mDoBacking)){
					track->clear();
                }else{
                    mObstracks = track;
                    setPatrolStatus(true);
                }
            }
        }
        mDetectingPiling = false;
    }

    void batteryStatusCallback(const statusConstPtr &s)
    {
        mCurBatPercentage=s->status[1];
        if (mPrevBatPercentage>=mMinBat
           && mCurBatPercentage<mMinBat){
            mLowBatFlag = true;
        }else if (mCurBatPercentage>=mMinBat){
            mLowBatFlag = false;
        }

        static bool bBack = false;

        mCharging = s->status[2];
		if (mCharging){
            bBack = false;
			mPrevBatPercentage = 100;
		}else  if(mLowBatFlag){
            if (mIsPatroling  && !bBack /*&& !mBCalibByAkaze*/){
                if (mTrace.getTracePercent()>0.5){
					PLOG_INFO(NAV_PATH_NODE_TAG,"set new TrackList");
                    auto track=make_shared<TrackList>();
                    *track=mCurrent;
                    mTrace.setTrackList(track);
                    bBack = true;
                }
            }
        	mPrevBatPercentage = mCurBatPercentage;
        }else{
        	mPrevBatPercentage = mCurBatPercentage;
		}
    }

	void goingHomeStatus(const std_msgs::Int32::ConstPtr& msg)
    {
        switch (msg->data)
        {
        case status::BACK_UP_SUCCESS:{
            mBackingHome = false;
        }
            break;
        case status::BACK_UP_FAIL:
        case status::BACK_UP_INACTIVE:
            mBackingHome = false;
            break;
        case  status::BACK_UP_DETECT: {
            mBackingHome = true;
        }
        break;
        case  status::BACK_UP_ALIGN:
        case  status::BACK_UP_BACK:
            mBackingHome = true;
            break;
        default:
            break;
        }
    }

    void test(const std_msgs::Int32::ConstPtr& msg)
    {
        // PLOG_INFO(NAV_PATH_NODE_TAG,"test %d",mIsPatroling);
        switch (msg->data){
            case 0:
            if (mIsPatroling){
                auto track=make_shared<TrackList>();
				*track=mCurrent;
				track->erase(0.3);
                mTrace.setTrackList(track);
            }
            break;
            default:
            break;
        }
    }

	double avg_angle_nomod(double *angAry, int len)
	 {
		 int i = 0;
		 double sum1 = 0.0;
		 for (i=0 ;i< len;i++)
		 {
			 sum1 += angAry[i];
		 }
		 sum1 /= len;

		 return sum1;
	 }

	#ifdef 	NO_IMU_FILTER_NODE		///<<< USE_SIMPLE_YAW_ESTIMATION

	 bool imuCalibrate(const ImuMsg::ConstPtr& msg)
	 {
///		 static const Vector3d G(0.0,0.0,G_VALUE);
		 static Vector3d v_acc_offset(9.0,9.0,9.0);
		 static Vector3d v_gyro_offset(9.0,9.0,9.0);
		 Vector3d accel(msg->linear_acceleration.x,msg->linear_acceleration.y,msg->linear_acceleration.z);
		 Vector3d gyro(msg->angular_velocity.x,msg->angular_velocity.y,msg->angular_velocity.z);

		 if (0 == m_calibrate_cnt)	 ///<<< Reset acc and gyro offset...
		 {
			 v_acc_offset  =Vector3d(0.0,0.0,0.0);
			 v_gyro_offset =Vector3d(0.0,0.0,0.0);
			 clog << "Gyro and accl offset reset:" <<endl;
			 clog << "Gyro:"<< endl << v_gyro_offset << endl;
			 clog << "Accl:"<< endl << v_acc_offset << endl;
			 fLog << "Gyro and accl offset reset:" <<endl;
			 fLog << "Gyro:"<< endl << v_gyro_offset << endl;
			 fLog << "Accl:"<< endl << v_acc_offset << endl;
		 }

		 if(m_calibrate_cnt++<mCALIBRATE_MAX_CNT){
			 if (m_calibrate_cnt > mCALIBRATE_SKIP_CNT)
			 {
				 v_acc_offset  += (accel-G);
				 v_gyro_offset += gyro;
			 }

			 if(m_calibrate_cnt==mCALIBRATE_MAX_CNT){
				 v_acc_offset /= (mCALIBRATE_MAX_CNT-mCALIBRATE_SKIP_CNT);
				 v_gyro_offset /= (mCALIBRATE_MAX_CNT-mCALIBRATE_SKIP_CNT);

				 m_acc_offset.x = v_acc_offset.x();
				 m_acc_offset.y = v_acc_offset.y();
				 m_acc_offset.z = v_acc_offset.z();
				 m_gyro_offset.x = v_gyro_offset.x();
				 m_gyro_offset.y = v_gyro_offset.y();
				 m_gyro_offset.z = v_gyro_offset.z();

				 std::cout<< endl <<"gyro offset:\n"<<m_gyro_offset<<std::endl;
				 std::cout<<"acc offset:\n"<<m_acc_offset<<std::endl;

				 mImuCalibrated = true;
			 }
			 return true;
		 }
		 return false;
	 }
	 void imuCallback(const ImuMsg::ConstPtr& imu_msg_raw)
	 {
		 if (false == mFilterRunning)
			 return;

		 int N_short =2 , N_long = 128;
		 double Vdc_residue_limit = 0.002;
		 static double avgDC_short = 0.0;
		 static double avgDC_long = 0.0;
		 static double prev_data = 0.0;
		 double delta_GyroZ = 0.0;

        //FIXME: How do lock here but do not have unlock????
		 boost::mutex::scoped_lock lock(mutex_);

#if TEST_SAVE_FILE
		  /// logIndex %= LOG_ARRAY_SIZE；
		  logArray[logIndex].header.seq = m_calibrate_cnt;
		  logArray[logIndex].header.stamp = imu_msg_raw->header.stamp;
		  logArray[logIndex].twist.linear.x = imu_msg_raw->angular_velocity.z;
#endif

		 if(imuCalibrate(imu_msg_raw)){
			 prev_data = 0.0;
			 avgDC_short =0.0;
			 avgDC_long = 0.0;
			 last_time_  = imu_msg_raw->header.stamp;
#if TEST_SAVE_FILE
			 logCount ++ ;	 /// logIndex ++;
			 logIndex = logCount % LOG_ARRAY_SIZE;
			 memset(&logArray[logIndex],0,sizeof(logArray[logIndex]));	 /// clear buffer
#endif
			 return;
		 }

		 geometry_msgs::Vector3 ang_vel = imu_msg_raw->angular_velocity;
		 geometry_msgs::Vector3 lin_acc = imu_msg_raw->linear_acceleration;

#if TEST_SAVE_FILE

		 logArray[logIndex].header.seq = m_calibrate_cnt;
		 logArray[logIndex].header.stamp = imu_msg_raw->header.stamp;
		 logArray[logIndex].twist.linear.x = ang_vel.z;
#endif

		 ang_vel.x -= m_gyro_offset.x;
		 ang_vel.y -= m_gyro_offset.y;
		 ang_vel.z -= m_gyro_offset.z;
		 lin_acc.x -= m_acc_offset.x;
		 lin_acc.y -= m_acc_offset.y;
		 lin_acc.z -= m_acc_offset.z;

#if TEST_SAVE_FILE
		  logArray[logIndex].twist.linear.y = ang_vel.z;
#endif

		 double data = ang_vel.z;
		 avgDC_short = avgDC_short - avgDC_short/N_short + data/N_short;
		 delta_GyroZ = avgDC_short - ( data - prev_data );
		 prev_data = data;

		 if (delta_GyroZ > Vdc_residue_limit)
			 delta_GyroZ = Vdc_residue_limit;
		 ///else
		 if (delta_GyroZ < -Vdc_residue_limit)
			 delta_GyroZ = -Vdc_residue_limit;

		 avgDC_long = avgDC_long - avgDC_long/N_long + delta_GyroZ/N_long;
		 ang_vel.z = data - avgDC_long;

		 ang_vel.x = 0.0;
		 ang_vel.y = 0.0;

		 lin_acc.x = 0.0;
		 lin_acc.y = 0.0;
		 lin_acc.z = G.z();

#if TEST_SAVE_FILE
		  logArray[logIndex].twist.linear.z = ang_vel.z;
#endif

		 ros::Time time = imu_msg_raw->header.stamp;

		 imu_frame_ = imu_msg_raw->header.frame_id;


		 if (!initialized_)
		 {

			 // initialize time
			 last_time_ = time;
			 initialized_ = true;
		 }

		 // determine dt: either constant, or from IMU timestamp
		 float dt;
		 if (constant_dt_ > 0.0)
			 dt = constant_dt_;
		 else
		 {
			 dt = (time - last_time_).toSec();
			 if (time.isZero())
				 ROS_WARN_STREAM_THROTTLE(
					 5.0,
					 "The IMU message time stamp is zero, and the parameter "
					 "constant_dt is not set!"
						 << " The filter will not update the orientation.");
		 }

		 last_time_ = time;

#ifdef  USE_SIMPLE_YAW_ESTIMATION
	  if(initialized_)
	  {
	    double currYaw = 0.0;

#ifdef YAW_USE_AVERAGE
		{
		 m_Yaw_Save += ang_vel.z * dt;

		 currYaw = m_Yaw_Save;	 ///m_Yaw_Pub ;

		 mYawBuff[mBufIndex] = currYaw;
		 mBufIndex ++;
		 mBufIndex %=  mYawAverCount; ///20;

		 m_Yaw_Pub = avg_angle_nomod(mYawBuff,mYawAverCount);	/// 20
		}

#else
		{
		 currYaw = m_Yaw_Save;  	/// m_Yaw_Pub ;

		 m_Yaw_Save += ang_vel.z * dt;

         m_Yaw_Pub = m_Yaw_Save;
		}
#endif
        geometry_msgs::TransformStamped transform;
        {
            transform.header.stamp = imu_msg_raw->header.stamp;
            transform.header.seq = m_calibrate_cnt;

            transform.header.frame_id = fixed_frame_;
            transform.child_frame_id = imu_frame_;

            transform.transform.translation.x = 0.0;
            transform.transform.translation.y = 0.0;
            transform.transform.translation.z = m_Yaw_Pub;

            transform.transform.rotation.w = cos(m_Yaw_Pub/2.0);
            transform.transform.rotation.x = 0.0;
            transform.transform.rotation.y = 0.0;
            transform.transform.rotation.z = 1.0*sin(m_Yaw_Pub/2.0);
        }
		/// transform_publisher_.publish(transform);
	 	imuFilterTransformCallback(transform);
#if TEST_SAVE_FILE
        if(transform.header.stamp != logArray[logIndex].header.stamp )
        {
            logCount ++;  ///logIndex ++;
            logIndex =  logCount % LOG_ARRAY_SIZE;
            memset(&logArray[logIndex],0,sizeof(logArray[logIndex]));   /// clear buffer
        }
#endif
#if TEST_SAVE_FILE
        logArray[logIndex].header.seq = m_calibrate_cnt;
        logArray[logIndex].header.stamp = transform.header.stamp;
        logArray[logIndex].twist.angular.z = m_Yaw_Pub;
        logArray[logIndex].twist.angular.x = currYaw;
        logArray[logIndex].twist.angular.y = m_Yaw_Pub - currYaw;

        logCount ++ ; /// logIndex++;
        logIndex = logCount % LOG_ARRAY_SIZE;
        memset(&logArray[logIndex],0,sizeof(logArray[logIndex]));  /// clear buffer
#endif

	  }
#endif
	 }

    void imuFilterTransformCallback(geometry_msgs::TransformStamped transform)
    {
        /// clog <<  rpy.header.stamp << endl;
            // ROS_INFO("++++++++++++++++++++++ %lf",rpy.header.stamp);
        if (mBImuPatrolCalib || mBCalibByAkaze){
            // PLOG_WARN(NAV_PATH_NODE_TAG,"(mBImuPatrolCalib || mBCalibByAkaze = %d || %d )",mBImuPatrolCalib , mBCalibByAkaze);
            return;
        }

        if (false == mGotImuYawOffset)
        {
            mImuYawOffset= transform.transform.translation.z;
            mGotImuYawOffset = true;
            ///<<< mImuFilterYaw = -1;
            PLOG_WARN(NAV_PATH_NODE_TAG,"Got mImuYawOffset: %f (Degree：%f)",mImuYawOffset, tf2Degrees(mImuYawOffset));
        }
        else
        {
            mImuFilterYaw = tf2Degrees(transform.transform.translation.z-mImuYawOffset);
            // PLOG_INFO(NAV_PATH_NODE_TAG,"imuRpyFilter(time: %lf , (roll,pitch,yaw): %10lf %10lf %10lf ) mImuFilterYaw = %f ",
            // transform.header.stamp.toSec(),
            // transform.transform.translation.x,transform.transform.translation.y,transform.transform.translation.z,mImuFilterYaw);
        }

#ifdef D_USE_IMU_FILTER_POSE
        mCurPose = Eigen::Quaterniond(transform.transform.rotation.w, 0.0 , 0.0 ,transform.transform.rotation.z);
        // PLOG_DEBUG(NAV_PATH_NODE_TAG,"ImuStamp: %lf ,mCurPose = ( %lf %lf %lf %lf) \n",transform.header.stamp.toSec(),
            // mCurPose.w(),mCurPose.x(),mCurPose.y(),mCurPose.z());
#endif

 }

#if TEST_SAVE_FILE
bool logOneRecord(int recNo,TwistStamped record, TwistStamped prevRec )
{
	//if(!imuLog.is_open())
	//	return false;
	if (0 == record.header.seq)  ///Imu Filter Start
	{
		if(imuLog.is_open())
		{
			imuLog.flush();
			imuLog.close();
		}
		fileCount ++;
		sprintf(logFileName,"/run/log/imuLog_%03d.txt",fileCount);
		imuLog.open(logFileName);
		if(!imuLog.is_open())
			return false;
	}
	imuLog << recNo << tab << record.header.seq <<  tab << fixed <<record.header.stamp << tab << (record.header.stamp -prevRec.header.stamp)
		<< tab << setw(13) << record.twist.linear.x << tab << record.twist.linear.y << tab <<record.twist.linear.z
		<< tab	<< record.twist.angular.x << tab << record.twist.angular.y << tab <<record.twist.angular.z << tab << tf2Degrees(record.twist.angular.z)
		<< cr << endl;

	if(!imuLog.is_open())
		return false;
	return true;
}
#endif

bool imuFilterCalibStatus()
{
	// ROS_INFO("call imuFilterCalibStatus %d", m_calibrate_cnt);
	bool ret = (m_calibrate_cnt < mCALIBRATE_MAX_CNT);
	return ret ;
}

bool imuFilterReset()
{
    // ROS_INFO("call imuFilterReset");
	mImuCalibrated = false;
    m_calibrate_cnt=0;
	/// m_gyro_offset = Vector3d(0.0, 0.0, 0.0);
	/// m_acc_offset = Vector3d(0.0, 0.0, 0.0);
	m_acc_offset.x = 0.0;
	m_acc_offset.y = 0.0;
	m_acc_offset.z = 0.0;
	m_gyro_offset.x = 0.0;
	m_gyro_offset.y = 0.0;
	m_gyro_offset.z = 0.0;
	m_Yaw_Pub = 0.0;
	m_Yaw_Save = 0.0;
#ifdef YAW_USE_AVERAGE
///<<<aaa
	mBufIndex =0;
	memset(mYawBuff,0,sizeof(mYawBuff));
#endif
#if TEST_SAVE_FILE
/// Save only one log...
	logCount = 0;
	logIndex = 0;
	memset(&logArray[logIndex],0,sizeof(logArray[logIndex]));	/// clear buffer
#endif
	return true;
}
bool imuFilterStart()
{
	mFilterRunning = true;
	// ROS_INFO("IMU Filter is runnig ... ");
	return true;
}
bool imuFilterStop()
{
	mFilterRunning = false;
	// ROS_INFO("IMU Filter is stopped ... ");
#if TEST_SAVE_FILE
		if(logCount > 0 )
		{

			int i =0;
			if(logCount > LOG_ARRAY_SIZE)  ///wrap around
				for (i=logIndex;i<LOG_ARRAY_SIZE;i++)
			{
	           logOneRecord(i, logArray[i], logArray[i-1]);
			}

		///	imuCab.flush();

			for (i=0;i<logIndex;i++)
			{
				if ( 0 == i)
				{
					if (logCount > LOG_ARRAY_SIZE)
						logOneRecord(i, logArray[i], logArray[LOG_ARRAY_SIZE-1]);
					else
						logOneRecord(i, logArray[i], logArray[i]);
				}
				else
					logOneRecord(i, logArray[i], logArray[i-1]);

			}
			imuLog.flush();
			imuLog.close();
			logIndex = 0;   /// Reset save pointer
			memset(&logArray[logIndex],0,sizeof(logArray[logIndex]));   /// clear buffer
			return true;
		}

#endif
	return true;
}

#endif ///>>> NO_IMU_FILTER_NODE  /// USE_SIMPLE_YAW_ESTIMATOIN

 void imuFilterDataCallback(const sensor_msgs::Imu::ConstPtr& msg)
	{
	///		// ROS_INFO("---------------- %ld",ros::Time::now().toNSec());
			if (mBImuPatrolCalib || mBCalibByAkaze){
///				PLOG_WARN(NAV_PATH_NODE_TAG,"(mBImuPatrolCalib || mBCalibByAkaze = %d || %d )",mBImuPatrolCalib , mBCalibByAkaze);
				return;
			}
#ifdef SAVE_SENSOR_DATA
			if(g_imuDataFile==NULL){
				g_imuDataFile=fopen("imuData.txt","wb");
			}
			if(g_imuDataFile!=NULL){
				fprintf(g_imuDataFile,"imuData(time: %lf ,orient: %10lf %10lf %10lf %10lf ,  gyro: %10lf %10lf %10lf , accl: %10lf %10lf %10lf )\n",
					msg->header.stamp.toSec(),
					msg->orientation.w,msg->orientation.x,msg->orientation.y,msg->orientation.z,
					msg->angular_velocity.x,msg->angular_velocity.y,msg->angular_velocity.z,
					msg->linear_acceleration.x,msg->linear_acceleration.y,msg->linear_acceleration.z
					);
			}
			if(g_imuOrientFile==NULL){
				g_imuOrientFile=fopen("imuOrient.txt","wb");
			}
			if(g_imuOrientFile!=NULL){
				fprintf(g_imuOrientFile,"%lf   %10lf %10lf %10lf %10lf \n",
					msg->header.stamp.toSec(),
					msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w
					);
			}
#endif
///			PLOG_WARN(NAV_PATH_NODE_TAG,"imuData(time: %lf ,orient:( %lf %lf %lf %lf ),gyro:\t %lf %lf %lf ,accl:\t %lf %lf %lf )",
///					msg->header.stamp.toSec(),
///					msg->orientation.w,msg->orientation.x,msg->orientation.y,msg->orientation.z,
///					msg->angular_velocity.x,msg->angular_velocity.y,msg->angular_velocity.z,
///					msg->linear_acceleration.x,msg->linear_acceleration.y,msg->linear_acceleration.z
///					);
			#ifdef D_USE_IMU_FILTER_POSE
				///<<<mCurPose = Eigen::Quaterniond(msg->orientation.w,msg->orientation.x,msg->orientation.y,msg->orientation.z);
				mCurPose = Eigen::Quaterniond(msg->orientation.w, 0.0 , 0.0 ,msg->orientation.z);
				PLOG_DEBUG(NAV_PATH_NODE_TAG,"ImuStamp: %lf ,mCurPose = ( %lf %lf %lf %lf) \n",msg->header.stamp.toSec(),
					mCurPose.w(),mCurPose.x(),mCurPose.y(),mCurPose.z());
			#endif
			///<<< cout << msg->header.stamp << endl;  //<<<   .toSec() ;
			///<<< cout << msg->orientation  ;

			///<<< cout << "imuDataMsg: " << *msg << endl;
	}

	void imuRpyFilterCallback(geometry_msgs::Vector3Stamped rpy)
	{
			if (mBImuPatrolCalib || mBCalibByAkaze){
///				PLOG_WARN(NAV_PATH_NODE_TAG,"(mBImuPatrolCalib || mBCalibByAkaze = %d || %d )",mBImuPatrolCalib , mBCalibByAkaze);
				return;
			}

#ifdef SAVE_SENSOR_DATA
			if(g_imuRpyFilterFile==NULL){
				g_imuRpyFilterFile=fopen("imuRpyFilter.txt","wb");
			}
			if(g_imuRpyFilterFile!=NULL){
				fprintf(g_imuRpyFilterFile,"imuRpyFilter(time: %lf , (roll,pitch,yaw): %10lf %10lf %10lf )\n",
					rpy.header.stamp.toSec(),
					rpy.vector.x,rpy.vector.y,rpy.vector.z);
			}
#endif
			if (false == mGotImuYawOffset)
			{
				mImuYawOffset= rpy.vector.z;
				mGotImuYawOffset = true;
				///<<< mImuFilterYaw = -1;
				PLOG_WARN(NAV_PATH_NODE_TAG,"Got mImuYawOffset: %f (Degree：%f)",mImuYawOffset, tf2Degrees(mImuYawOffset));
			}
			else
			{
				#ifdef D_USE_IMU_FILTER_POSE
				///	mSumW = rpy.vector.z;
				#endif

				mImuFilterYaw = tf2Degrees(rpy.vector.z-mImuYawOffset);
			PLOG_DEBUG(NAV_PATH_NODE_TAG,"imuRpyFilter(time: %lf , (roll,pitch,yaw): %10lf %10lf %10lf ) mImuFilterYaw = %f ",
				rpy.header.stamp.toSec(),
				rpy.vector.x,rpy.vector.y,rpy.vector.z,mImuFilterYaw);
			}
	}

#ifdef USE_MADGWICK_IMU_FILTER
    void imuMagCallback(const ImuMsg::ConstPtr& imu_msg_raw,const MagMsg::ConstPtr& mag_msg)
    {
        const geometry_msgs::Vector3& ang_vel = imu_msg_raw->angular_velocity;
        const geometry_msgs::Vector3& lin_acc = imu_msg_raw->linear_acceleration;
        const geometry_msgs::Vector3& mag_fld = mag_msg->magnetic_field;

        ros::Time time = imu_msg_raw->header.stamp;

        /*** Compensate for hard iron ***/
        geometry_msgs::Vector3 mag_compensated;
        mag_compensated.x = mag_fld.x;
        mag_compensated.y = mag_fld.y;
        mag_compensated.z = mag_fld.z;
        magCalibrate(mag_compensated.x,mag_compensated.y,mag_compensated.z,mMagCalibraParam);
        if (!mOrentianInited)
        {
            // PLOG_INFO(NAV_PATH_NODE_TAG,"First pair of IMU and magnetometer messages received.");
        }

        if (!mOrentianInited)
        {
            // wait for mag message without NaN / inf
            if(!std::isfinite(mag_fld.x) || !std::isfinite(mag_fld.y) || !std::isfinite(mag_fld.z))
            {
                return;
            }

            geometry_msgs::Quaternion init_q;
            StatelessOrientation::computeOrientation(mWorldFrame, lin_acc, mag_compensated, init_q);
            mIMUFilter->setOrientation(init_q.w, init_q.x, init_q.y, init_q.z);

            mLastTime = time;
            mOrentianInited = true;
        }

        float dt=(time - mLastTime).toSec();
        mLastTime = time;
        mIMUFilter->madgwickAHRSupdate(
            ang_vel.x, ang_vel.y, ang_vel.z,
            lin_acc.x, lin_acc.y, lin_acc.z,
            mag_compensated.x, mag_compensated.y, mag_compensated.z,
            dt);

        if(!mPubIMUPose){
            return;
        }
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id="world";
        pose.header.stamp=time;

        double q0,q1,q2,q3;
        mIMUFilter->getOrientation(q0,q1,q2,q3);
        pose.pose.orientation.w=q0;
        pose.pose.orientation.x=q1;
        pose.pose.orientation.y=q2;
        pose.pose.orientation.z=q3;

        mIMUPose.publish(pose);
    }
#endif
    ros::NodeHandle mGlobal;
    ros::NodeHandle mLocal;
#ifdef IMU_USE_QUEUE_THREAD
    ros::CallbackQueue imuCallbackQueue;
#endif
    ros::ServiceClient  mDetectRecordStatusClient;
    ros::ServiceClient  mImuPatrolCalibClient;
    ros::ServiceClient  mImuPatrolCalibStatusClient;
    ros::ServiceClient  mImuFilterCalibStatusClient;
    ros::ServiceClient  mSaveTmpPicClient;
    ros::ServiceClient  mGetDiffAngleClient;
	ros::ServiceClient  mImuFilterResetClient;
	ros::ServiceClient  mImuFilterStartClient;
	ros::ServiceClient  mImuFilterStopClient;

    ros::ServiceServer mStart;
    ros::ServiceServer mSave;
    ros::ServiceServer mPatrol;
    ros::ServiceServer mCancel;
    ros::ServiceServer mPatrolStop;
    ros::ServiceServer mListPath;
    ros::ServiceServer mDeletePath;
    ros::ServiceServer mMagClibra;
    ros::ServiceServer mGetName;
    ros::ServiceServer mGetStatus;
    ros::ServiceServer mAddWayPt;
    ros::ServiceServer mQueryWayPt;
    ros::ServiceServer mEnableVioSrv;
	ros::ServiceServer mGetCalibStatusSrv;
	ros::ServiceServer mLowBatSrv;
	ros::ServiceServer mExitSrv;
    ros::Subscriber mOdomRelative;
    ros::Subscriber mTof;
    ros::Subscriber mDetectPileSub;
    ros::Subscriber mImuFilterDataSub;
    ros::Subscriber mImuFilterRpySub;
	ros::Subscriber mImuFilterTransSub;

	ros::Subscriber mGoingHomeStatusSub;
    ros::Publisher mPose;
    ros::Publisher mCmdVel;
    ros::Publisher mNavTraceDonePub;
    atomic<bool> mDoingMagCalibra;

	bool mGotImuYawOffset;
	float mImuYawOffset;
	float mImuFilterYaw;
#ifdef USE_SIMPLE_YAW_ESTIMATION
#ifdef NO_IMU_FILTER_NODE
	int m_calibrate_cnt = 0;
	int mCALIBRATE_MAX_CNT = 500;
    int mCALIBRATE_SKIP_CNT = 300;
	geometry_msgs::Vector3 m_acc_offset ;
	geometry_msgs::Vector3 m_gyro_offset;
	bool mImuCalibrated=false;
	int mLogCycleCount = 20;
	double m_Yaw_Save = 0.0;
	double m_Yaw_Pub = 0.0;
	ofstream fLog;
    boost::mutex mutex_;
    std::string fixed_frame_;
    std::string imu_frame_;
	bool mFilterRunning = false;
	bool initialized_ = false;
	double constant_dt_ = 0.0;
    ros::Time last_time_;
    ros::Subscriber mImuDataSub;
#ifdef IMU_USE_QUEUE_THREAD
    ros::SubscribeOptions imuSubOps;
    std::thread imuRosQueueThread;
#endif
#ifdef YAW_USE_AVERAGE
	int mBufIndex = 0;
	double mYawBuff[32];	   /// 20
	int mYawAverCount = 20;		/// mCfg.getImuFilterYawAverageCount(); 		   ///=20;
#endif
#endif
#endif
#ifdef USE_MADGWICK_IMU_FILTER
    std::shared_ptr<ImuSubscriber> mIMUSub;
    std::shared_ptr<MagSubscriber> mMagSub;
    std::shared_ptr<Synchronizer> mSync;
    std::shared_ptr<ImuFilter> mIMUFilter;
    MagCalibraParam mMagCalibraParam;
    float mIMUFilterGain;
    float mIMUFilterZeta;
    ros::Publisher mIMUPose;
    bool mPubIMUPose;
    bool mOrentianInited;
    WorldFrame::WorldFrame mWorldFrame;
    ros::Time mLastTime;
#endif
    boost::mutex mutex_OBS;
    boost::mutex mutex_calcDistAngle;
	boost::mutex mutex_trace;

	bool mBImuPatrolCalib = false;
	bool mBCalibByAkaze = false;
	bool mBCalibByImuFilter = false;
	bool mObsStatus = false;
	bool mPlayingCalibAudio = false;
	TrackList mObsCurrent;

	shared_ptr<TrackList> mObstracks;

	double mTofDistance = 0.0;
	double mLastObsDistance = 0.0;

	//end rmwei
    TrackList mCurrent;
    TrackTrace mTrace;
    Eigen::Quaterniond mCurPose;
    Eigen::Vector3d  mCurPosition;
    string mPatrolingPathName;
    bool mIsTracing;                                //is move by path
    bool mDoBacking;
    bool mPathPlanning=false;
    double mLastTraceStamp;
    bool mIsPatroling;                                //patrol state
    bool mDetectingPiling;                          //detect chargepile when return by one key
    bool mIsCancel;
    bool mDetectedPile;
	bool mCancelDetectPile = false;
    AlgoUtils mAlgo;
    atomic<bool> mAdjusting;

    StatusPublisher mPub;

    atomic<int> mDetectObstacleCnt;
    DistanceCloud mDistCloud;
    bool mClipData;

    BackingUpHelper mBackUp;
    ros::Publisher mSysEvtPub;
    ros::Publisher mPatrolStatusPub;
    ros::Publisher mStatusPub;

    ros::Subscriber mTestNav;
    ros::Timer mTimer;

    DeviceDefaultConfig mCfg;

    int mMinBat;
    bool mLowBatFlag;
    bool mIsStartFromOut;
    int mPrevBatPercentage;
    int mCurBatPercentage;
    int mMaxPathSize;
    ros::Subscriber mBatteryStatus;
    ros::Subscriber mObjDetect;
    int mTofAvgCnt;
    float mAvgTof;
    bool mBUseVio;
	int mTraceCounter;
	double mSumW;
	bool mBackingHome = false;
    float mObsWz;
    bool mCharging = false;

    // New obs avoidance
    bool isObs = true;
    int mObsCnt = 0;
    bool stoppedAvoidObs;
    bool mTriedReturnHomeOnce = false;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "NavPathNode");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,NAV_NODE_DEBUG_LEVEL);
  NavigatePath nav;
  ros::spin();

#ifdef SAVE_SENSOR_DATA
    // ROS_INFO("save imu data...\n");
  if(g_imuRpyFilterFile!=NULL){
	  fclose(g_imuRpyFilterFile);
  }
#endif
 return 0;
}
