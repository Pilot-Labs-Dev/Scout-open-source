#include<memory>
#include<thread>
#include<atomic>
#include"ros/ros.h"
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


using namespace std;
using namespace roller_eye;

#define NAV_PATH_NODE_TAG           "nav_path_node"
//#define NAV_PATH_MAX_SIZE               20000
#define NAV_PATH_MAX_SIZE               100000

#define NAV_OBSTACLE_MIN_DIST                   0.2
#define NAV_OBSTACLE_CNT                               5
#define NAV_OBSTACLE_MAX_DIST                   0.5
#define NAV_MANUALSAVE_MIN_DIST             2.0          //min dist of manual save when schedule path
#define NAV_OBSTACLE_FORWARD_DIST       0.4
//#define NAV_OBSTACLE_ANGLE              (10.0*M_PI/180)
#define NAV_OBSTACLE_ANGLE              (30.0*M_PI/180)

#define ENALBE_AVOID_OBSTACLE 
//#define USE_MADGWICK_IMU_FILTER   
#define MAG_CALIBRATE_CNT               100
#define NAV_ERASE_CNT                           10

#define RETRUN_AVOID_IFCANCEL    if (mIsCancel) return;

#define TRACK_ROLL_SPEED							1.1
#define TRACK_MOVE_SPEED							0.15

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
    OBSTACLE_MIN_DISTANCE(mCfg.getObstacleMinDistance()),
	mTraceCounter(0),
	mSumW(0.0)
    {
    	//rmwei
		log_t arg = {
			confpath:	"/var/roller_eye/config/log/" NAV_PATH_NODE_TAG ".cfg",
			levelpath:	"/var/roller_eye/config/log/log.level",
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
        PltConfig::getInstance()->getMinBatForPowerOff(mMinBat);
        VioParam param;
        load_vio_param(param);
        mBUseVio = param.enable;
        mk_depth_dir(NAVIGATE_PATH_PATH);
        mImuPatrolCalibClient = mLocal.serviceClient<imu_patrol_calib>("/imu_patrol_calib");
        mImuPatrolCalibStatusClient = mLocal.serviceClient<getimu_patrolcalib_status>("/getimu_patrolcalib_status");
        mDetectRecordStatusClient = mLocal.serviceClient<detect_record_get_status>("/DetectRecordNode/get_status");
		mSaveTmpPicClient = mLocal.serviceClient<saveTmpPicForStartPath>("/CoreNode/saveTmpPicForStartPath");
		mGetDiffAngleClient = mLocal.serviceClient<getDiffAngleWhenPatrol>("/CoreNode/getDiffAngleWhenPatrol");
		
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
        mPose=mLocal.advertise<geometry_msgs::PoseStamped>("pose",100);
        mCmdVel=mGlobal.advertise<geometry_msgs::Twist>("cmd_vel",100);
        mNavTraceDonePub = mGlobal.advertise<std_msgs::Int32>("/nav_trace_done",1);
        mSysEvtPub = mGlobal.advertise<std_msgs::Int32>("/system_event",1);
        mPatrolStatusPub = mGlobal.advertise<patrol_status>("/patrol_status",1);
        mBatteryStatus  = mGlobal.subscribe("SensorNode/simple_battery_status", 
                                                                                                            10, &NavigatePath::batteryStatusCallback,this);
		mTimer=mGlobal.createTimer(ros::Duration(1),&NavigatePath::timerCallback,this);
		mTimer.stop();

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

    }
    ~NavigatePath()
    {
		dzlogfInit();
    }
private:

    void initEstimate()
    {
        PLOG_DEBUG(NAV_PATH_NODE_TAG,"init estimate");
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

#ifdef ENALBE_AVOID_OBSTACLE
        if(!mTof){
            mTof=mGlobal.subscribe("SensorNode/tof",1,&NavigatePath::tofDataCB,this);
        } 
		if(!mObsTof){
			mObsTof=mGlobal.subscribe("SensorNode/tof",1,&NavigatePath::ObstofCB,this);
		}
#endif
    }
    void deinitEstimate()
    {
		mSumW = 0.0;
		mTraceCounter = 0;
		mBCalibByAkaze = false;
        mPatrolingPathName = "";
        mOdomRelative.shutdown();
#ifdef ENALBE_AVOID_OBSTACLE
        mTof.shutdown();
        mObsTof.shutdown();
#endif
#ifdef USE_MADGWICK_IMU_FILTER 
        mSync.reset();
        mMagSub.reset();
        mIMUSub.reset();
        mIMUFilter.reset();
#endif
        PLOG_DEBUG(NAV_PATH_NODE_TAG,"deinit estimate");
    }
    bool start(bool reset)
    {
        mIsCancel = false;
        mDetectedPile = false;
		mDetectingPiling = false;
        if(reset){
            mCurPose=Eigen::Quaterniond(1.0,0.0,0.0,0.0);
            mCurPostion=Eigen::Vector3d(0.0,0.0,0.0);
            mCurrent.clear();
        }
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
        deinitEstimate();
        updateStatus();
    }
	bool getCalibrationStatus(nav_calibration_get_statusRequest& req,nav_calibration_get_statusResponse& res)
    {  
        res.ret = mBImuPatrolCalib|mBCalibByAkaze;
        return true;
    }
	void timerCallback(const ros::TimerEvent& evt)
    {
		static int counter = 0; 
		if (0==counter){
			if (!mPlayingCalibAudio){
				auto th=std::thread([this](){
					mPlayingCalibAudio = true;  
					playCalibAudioLoop();
				});
				th.detach();
			}
			if (!mBImuPatrolCalib){
				return;
			}
		}
		getimu_patrolcalib_status status;
		if (counter++<10 && mImuPatrolCalibStatusClient.call(status)){
			mBImuPatrolCalib = status.response.ret;
		}else{
			mBImuPatrolCalib = false;
		}
		if (!mBImuPatrolCalib && !mBCalibByAkaze){
	        system("sudo killall aplay");
			mTimer.stop();
			counter = 0;
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
            PLOG_INFO(NAV_PATH_NODE_TAG,"mag calibra is active");
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
		savePath.request.name = NAVIGATE_JPG_ROOT_PATH+pathName+"_1.jpg";
		mSaveTmpPicClient.call(savePath);		

		imu_patrol_calib ipcalib;
		mImuPatrolCalibClient.call(ipcalib);
		mTimer.start();
	}
    bool start_path(nav_path_startRequest& req,nav_path_startResponse& res)
    {
        if(mIsTracing){
            PLOG_WARN(NAV_PATH_NODE_TAG,"is tracing");
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
        ROS_INFO("call enableVio %d!\n", mBUseVio);
        return true;
    }


    bool save_path(nav_path_saveRequest& req,nav_path_saveResponse& res)
    {    
        if(mIsTracing){
            PLOG_WARN(NAV_PATH_NODE_TAG,"is patrolling");
            return false;
        }
        if(mCurrent.size()==0){
           PLOG_WARN(NAV_PATH_NODE_TAG,"path is empty");
            return false;            
        }
        bool ret=true;
        if(req.name.size()>0){//if req.name is empty string,only stop odom not save                       
			ret = save_path(req.name);
			std_msgs::Int32 status;
			status.data = 1;
			status.data = ret?0:1;
			mNavTraceDonePub.publish(status);
			return ret;           
        }
        stop();
        return ret;
    }

    bool save_path(string name)
    {
        bool isDoBacking = mDoBacking;
        bool isStartFromOut = mIsStartFromOut;
        PLOG_DEBUG(NAV_PATH_NODE_TAG,"call save_path2 %s! %d", 
                                           name.c_str(),isStartFromOut);   
        bool ret=true;       
        string path=NAVIGATE_PATH_PATH+name;
        ret=(mCurrent.saveToFile(path)==0);
        if(!ret){
            PLOG_ERROR(NAV_PATH_NODE_TAG,"save path[%s] fail",path.c_str());
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
    	printf("stopPatrol !!!!!!!!\r\n");
        printf("mObstracks->size(%d) !!!!!!!!\r\n",mObstracks->size());
        if( (1 == misAvoidObs) && (true == mObsReturn) ){
			//printf("#Return start point ok.#\r\n\r\n");
			PLOG_ERROR(NAV_PATH_NODE_TAG,"stopPatrol #Return start point ok.#\r\n");
			//misAvoidObs = 1;
			ObsTryOtherSide();
			return true;
        }

        /*if(!mDoBacking)*/{ 
            stop();
        }
        mDoBacking=false;
        setPatrolStatus(false);
        
        if(1 == misObsBackHome){
			ObsBackHomeThread();
			return true;
        }
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
        double dist = trace.distanceFrontPoint(mCurPose, mCurPostion);
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
	void calibByAkaze(string pathName)
	{
		mTimer.start();

		getDiffAngleWhenPatrol diff;
		diff.request.name = NAVIGATE_JPG_ROOT_PATH+pathName+".jpg";		

		for (int i=0; i<1; i++){
			if (mGetDiffAngleClient.call(diff)){
				if (!rollForAkaze(diff.response.angle)){
					break;
				}
			}else {
				break;
			}
		}

		geometry_msgs::Twist twist;
		twist.linear.x=0;
		twist.linear.y=0.1;
		twist.angular.z=0;
		mCmdVel.publish(twist);
		usleep(500*1000);      
		twist.linear.y=0.0;
		mCmdVel.publish(twist);

		diff.request.name = NAVIGATE_JPG_ROOT_PATH+pathName+"_1.jpg";		
		for (int i=0; i<1; i++){
			if (mGetDiffAngleClient.call(diff)){
				if (!rollForAkaze(diff.response.angle)){
					break;
				}
			}else {
				break;
			}
		}
		mBImuPatrolCalib = true;
		imu_patrol_calib ipcalib;
		mImuPatrolCalibClient.call(ipcalib);
		mBCalibByAkaze = false;
	}

    bool patrol(nav_patrolRequest& req,nav_patrolResponse& res)
    {
        if(mIsTracing){
            PLOG_WARN(NAV_PATH_NODE_TAG,"is patrolling");
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
            PLOG_INFO(NAV_PATH_NODE_TAG,"back to home");
        }else{
            string path=NAVIGATE_PATH_PATH+req.name;
            mPatrolingPathName = req.name;
            if (mLowBatFlag){
                return false;
            }

            mIsStartFromOut = req.isFromOutStart;

            if(track->loadFromFile(path)<0){
                PLOG_ERROR(NAV_PATH_NODE_TAG,"load path[%s] fail",path.c_str());
                return false;
            }else{
                PltConfig::getInstance()->updateLastPatrolName(req.name);
            }
			mBCalibByAkaze = true;
            //track->erase(NAV_ERASE_CNT);
            track->erase(0.3);
			auto th=std::thread([this](){  
                    calibByAkaze(mPatrolingPathName);
                });
            th.detach();         
        }
		mTrace.setTrackList(track);
         if(!start(!mDoBacking)){
			track->clear();
            return false;
        }

        misAvoidObs = 0;
        mObsStartAngle = 0;
        mObsCurentStatus = 0;
        mIsObsMask.clear();
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
                PLOG_ERROR(NAV_PATH_NODE_TAG,"delete path[%s] fail",path.c_str());
                ret=false;
            }
			path = NAVIGATE_JPG_ROOT_PATH+name+".jpg";
            if(remove(path.c_str())!=0){
                PLOG_ERROR(NAV_PATH_NODE_TAG,"delete path jpg[%s] fail",path.c_str());
                ret=false;
            }
			path = NAVIGATE_JPG_ROOT_PATH+name+"_1.jpg";
            if(remove(path.c_str())!=0){
                PLOG_ERROR(NAV_PATH_NODE_TAG,"delete path jpg[%s] fail",path.c_str());
                ret=false;
            }
        }
        return ret;
    }
	
    void odomRelativeCB(const nav_msgs::Odometry::ConstPtr& msg)
    {		
		if (mBImuPatrolCalib || mBCalibByAkaze){
			return;
		}

        Eigen::Vector3d t(msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);
        float vx=msg->twist.twist.linear.x;
        float vy=msg->twist.twist.linear.y;
        float w=msg->twist.twist.angular.z;
        
        if (mBUseVio){
            Eigen::Quaterniond cp(msg->pose.pose.orientation.w,msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z);
            mCurPostion = t;
            mCurPose       = cp;
        }else{
            #ifndef USE_MADGWICK_IMU_FILTER
            //mCurPostion+=mCurPose*t;
            mCurPose=mCurPose*Eigen::Quaterniond(msg->pose.pose.orientation.w,msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z);
            mCurPostion+=mCurPose*t;
            #else
            if(!mOrentianInited){
                return;
            }
            mCurPostion+=mCurPose*t;
            mCurPostion.z()=0.0;
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
        
        pose.pose.position.x=mCurPostion.x();
        pose.pose.position.y=mCurPostion.y();
        pose.pose.position.z=mCurPostion.z();
        pose.pose.orientation.w=mCurPose.w();
        pose.pose.orientation.x=mCurPose.x();
        pose.pose.orientation.y=mCurPose.y();
        pose.pose.orientation.z=yaw;    
        mPose.publish(pose);
		
	    if (mDetectingPiling){
            return;
        }

        if(mCurrent.size()>mMaxPathSize){
            PLOG_DEBUG(NAV_PATH_NODE_TAG,"mMaxPathSize=%d %d", mMaxPathSize,mCurrent.size());
            stopPatrol();
            stop();
        }else{
            TrackPoint point(msg->header.stamp.toNSec(),mCurPostion,mCurPose,vx,vy,w);
            mCurrent.push(point);

            if( 1 == misAvoidObs){
		        ObsdoTrace(msg->header.stamp.toSec());
		        mObsCurrent.push(point);
            }else{
            	doTrace(msg->header.stamp.toSec());
            }
        }
    }
    void tofDataCB(const sensor_msgs::RangeConstPtr &r)
    {
        if(r->range>=0 && r->range<NAV_OBSTACLE_MIN_DIST){
            mDetectObstacleCnt++;
        }else{
            mDetectObstacleCnt=0;
        } 
    }
	void ObstofCB(const sensor_msgs::RangeConstPtr &r)
	{
		float range = 0;
		
		if(isinf(r->range)){
			range = 2.0;
		}else{
			range = r->range;
		}
		//mTofDistance = range;
		mAvgTof = mAvgTof+(range-mAvgTof)/mTofAvgCnt;
		mTofDistance=mAvgTof;
		//printf("ObstofCB:%f,%f\r\n",mTofDistance,r->range);
	}

    void doTrace(double stamp)
    {   
        if(!mIsTracing && !mDoBacking){
            return;
        }

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
        
        if(mDetectObstacleCnt>NAV_OBSTACLE_CNT){ 
            PLOG_DEBUG(NAV_PATH_NODE_TAG,"Detect obstacle");

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
            PLOG_INFO(NAV_PATH_NODE_TAG,"traceDone save_path");
            bool bRet = save_path(mPatrolingPathName);
            status.data = bRet?0:1;
            mNavTraceDonePub.publish(status);
        }

        mPathPlanning = false;
        if (!mIsPatroling || 
            (mIsPatroling && mIsStartFromOut)){          
            stopPatrol();           
            //rmwei
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
        mDetectedPile = true;        
    }

    void traceOnce()
    {
        mTrace.updatePose(mCurPostion,mCurPose);
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


        mAdjusting=true;
        auto th=std::thread([this](){
            avoidObstacle2();
            mAdjusting=false;
        });
        th.detach();

    }
    //rmwei add .
    #define TRACK_MOVE_DIS_PONIT                       0.3
    #define TRACK_MOVE_REPEAT						   2
    #define TRACK_MOVE_ANGLE_OFFET	                   15*M_PI/180
    #define TRACK_DIRECTION_RIGHT						-1
    #define TRACK_DIRECTION_LEFT						1

	#define TRACK_PATH_MOVE_OFFSET					   	0.5
	#define TRACK_PATH_ANGLE_OFFSET					   	10*M_PI/180
	
	#define TRACK_LOOK_DIS_PONIT					   (TRACK_PATH_MOVE_OFFSET+TRACK_MOVE_DIS_PONIT/2)
	//#define TRACK_LOOK_DIS_PONIT						0.5

	#define TRACK_DIRECTION_FORWARD						1
	#define TRACK_DIRECTION_BACKWARD					-1

	typedef enum {
		E_OBS_DIRLEFT		= 1,
		E_OBS_DIRRIGHT,
		E_OBS_RETURN,
		E_OBS_BACKHOME,
		E_OBS_BACKUP
	} OBS_STATUS;
	 void ObsTryOtherSideThread()
	 { 		 
		 auto th=std::thread([this](){
			 ObsTryOtherSide();			 
			 mAdjusting=false;
		 });
		 th.detach();
 	 }

	 bool ObsStopPatrol()
	 { 
		 PLOG_ERROR(NAV_PATH_NODE_TAG,"===>stopPatrol !!!!!!!!\r\n");
		 PLOG_ERROR(NAV_PATH_NODE_TAG,"===>mObstracks->size(%d) !!!!!!!!\r\n",mObstracks->size());
		 PLOG_ERROR(NAV_PATH_NODE_TAG,"==>mObsTrackOffet.size:%d,mObsLastTracks.size:%d,mTrace.size(%d),mObstracks.size(%d),mObsCurrent.size(%d),mObsHome.size(%d)",
		 mObsTrackOffet.size(),mObsLastTracks.size(),mTrace.size(),mObstracks->size(),mObsCurrent.size(),mObsHome.size());

		 if( (1 == misAvoidObs) && (E_OBS_RETURN == mObsCurentStatus) ){
			 //printf("#Return start point ok.#\r\n\r\n");
			 PLOG_ERROR(NAV_PATH_NODE_TAG,"===>stopPatrol #Return start point ok.#\r\n");

			 ObsTryOtherSide(0);
			 return true;
		 }
 
		//  stop();
		//  mDoBacking=false;
		//  setPatrolStatus(false);
		 
		 if(E_OBS_BACKHOME == mObsCurentStatus){
			 ObsBackHomeThread();
			 return true;
		 }
		 return true;
	 }


	 void ObsTraceOnce()
	 {
		 mTrace.updatePose(mCurPostion,mCurPose);
	 
		 float vx,vy,w,roll;
		 PLOG_DEBUG(NAV_PATH_NODE_TAG,"mTrace.traceOnce %d",mTrace.size());
		 if(mTrace.traceOnce(vx,vy,w,roll)){
			 PLOG_DEBUG(NAV_PATH_NODE_TAG,"doTrace	startRoll:%f",roll);
			 startRoll(roll);
		 }else{
			 geometry_msgs::Twist twist;
			 twist.linear.x=vx;
			 twist.linear.y=vy;
			 twist.angular.z=w;
			 mCmdVel.publish(twist);
			 PLOG_DEBUG(NAV_PATH_NODE_TAG,"vx:%f vy:%f w:%f",vx,vy,w);
		 }
	 }

	 void ObsTraceDone()
	 {
		 PLOG_DEBUG(NAV_PATH_NODE_TAG,"ObsTraceDone %d %s", mIsPatroling,mPatrolingPathName.c_str());
		//  if (!mIsPatroling && !mPatrolingPathName.empty()){
		// 	 std_msgs::Int32 status;
		// 	 status.data = 1;
        //     PLOG_INFO(NAV_PATH_NODE_TAG,"ObsTraceDone save_path");
		// 	 bool bRet = save_path(mPatrolingPathName);
		// 	 status.data = bRet?0:1;
		// 	 mNavTraceDonePub.publish(status);
		//  }
 
		//  mPathPlanning = false;
		 if (!mIsPatroling || 
			 (mIsPatroling && mIsStartFromOut)){		  
				 PLOG_DEBUG(NAV_PATH_NODE_TAG,"traceDone mIsPatroling:%d  bIsStartFromOut:%d %s %d",
												   mIsPatroling, mIsStartFromOut,  __FILE__, __LINE__);
				 ObsStopPatrol();
				 
				 PLOG_ERROR(NAV_PATH_NODE_TAG,"ObstraceDone...");
				 //rmwei
//	 		   auto th=std::thread([this](){  
//	 				   mBackUp.begin(mGlobal);
//	 			   });
//	 		   th.detach(); 		
		 }else{
			 ObsStopPatrol();	   
		 }
	 }

	 void ObsdoTrace(double stamp)
	 {
		 if(!mIsTracing && !mDoBacking/*||stamp-mLastTraceStamp<0.1*/){
			 return;
		 }
	 
		 //PLOG_DEBUG(NAV_PATH_NODE_TAG,"doTrace %d %d", mIsPatroling,mTrace.size());
		 if(mTrace.done() || mDetectedPile){		 
			 ObsTraceDone(); //rmwei
			 return;
		 }
		 if (mIsPatroling && isDetectRecord()){
			 return;
		 }
		 
		 mLastTraceStamp=stamp;
		 
		 if(mAdjusting){
			 return;
		 }
		 
		 if(mDetectObstacleCnt>NAV_OBSTACLE_CNT){
		 //if(mAvgTof<OBSTACLE_MIN_DISTANCE){
			 PLOG_DEBUG(NAV_PATH_NODE_TAG,"Detect obstacle");
	 
			 std_msgs::Int32 event;
			 event.data = SYSEVT_OBSTACLE;
			 mSysEvtPub.publish(event);
	 
			 startAvoidObstacle();	 
			 return;
		 }
	 
		 ObsTraceOnce();
	 }

    
	 void calDistanceAndAngle(Eigen::Quaterniond& q,Eigen::Vector3d& pos,Eigen::Vector3d &goal,double &angle,double &distance)
		 {
			 Eigen::Vector3d d=q.inverse()*(goal-pos);
			 distance=d.norm();
			 if(distance<0.1/2){
				 angle=0;
				 //printf("calDistanceAndAngle !!!!!!\r\n");
				 return;
			 }
			 angle=std::asin(d.y()/distance);
//				cout <<"angle:" << angle
//					<<" d.x():" << d.x()
//					<<" d.y():" << d.y()
//					<<" distance:" << distance << endl;
			 if(d.x()<0){
				 angle=(angle>0?(M_PI-angle):(-M_PI-angle));
			 }
			 angle-=M_PI_2;
			 if(angle<-M_PI){
				 angle+=2*M_PI;
			 }
		 }

	bool catTwoPointDist(double &angle,double &distance){


#if 1 //rmwei new
			distance = -1;
			angle = 0;
			double lastDistance =0.0;
			int count  =0;

			//map<double, int> dict;
			shared_ptr<TrackList> ObstracksList = mTrace.getTrackList();
			auto pdict = make_shared<map<pair<double, double>, int>>();
			int index  = 0;
			auto iterB = ObstracksList->TrackPointList().rbegin();
			
			for (auto iter = ObstracksList->TrackPointList().rbegin(); iter != ObstracksList->TrackPointList().rend(); iter++){
				double tangleB =0;
				double tdistanceB = 0;
	
				calDistanceAndAngle(iterB->orientation,iterB->position,iter->position,tangleB,tdistanceB);
				if(tdistanceB > lastDistance ){
					double tangle =0;
					double tdistance = 0;
					calDistanceAndAngle(mCurPose,mCurPostion,iter->position,tangle,tdistance);
					//printf("TwoPoint mCurPostion(%f,%f) B(%f,%f),angle:%f,distance:%f\r\n",
					//mCurPostion.x(),mCurPostion.y(),iter->position.x(),iter->position.y(),tangle,tdistance);
						pair<double, double> DistPoint(tdistance, tangle);
						pair<pair<double, double>, int> p2(DistPoint,index);
						pdict->insert(p2/*pair<pair<double, double>, int>(DistPoint, index)*/); 
						auto mapiter = pdict->begin();
						// printf("index:%d,p2.second:%d\r\n",index,p2.second);
						// cout<<mapiter->first.first<<" "
						//	<<mapiter->first.second << " "
						//	<<mapiter->second<<endl;
					lastDistance = tdistanceB;
					mObsFurthestPointAngle = tangle;
				}else{
					if(10 < count++) break;
				}
				if(300 < (index++))break;
			}
			auto mapiter = pdict->begin();
			distance = mapiter->first.first;
			//distance = lastDistance;
			mObsFurthestPointDistance = lastDistance;
			angle = mapiter->first.second;
			mTrackListIndex = mapiter->second;
#endif

	}

	bool catObsEndPointAngle(){
		//shared_ptr<TrackList> & TrackTrace::getTrackList()
		shared_ptr<TrackList> ObstracksList = mTrace.getTrackList();
		if(ObstracksList->size()==0){
			return false;
		}
		mOBsEndPointAngle = 0;
		mOBsEndPointDistance =0;
		calDistanceAndAngle(mCurPose,mCurPostion,ObstracksList->front().position,mOBsEndPointAngle,mOBsEndPointDistance);
		if( (M_PI/2 < abs(mOBsEndPointAngle)) &&(abs(mOBsEndPointAngle) < M_PI*3/2) ){
			mOBsEndPointDirection = TRACK_DIRECTION_FORWARD;//
		}else{
			mOBsEndPointDirection = TRACK_DIRECTION_BACKWARD;//
		}		

		return true;
	}

	bool ObsFindNeedMovePoint(shared_ptr<TrackList> ptrack){
		double distance = 0;
		double angle = 0;
		int count =0;
		while(ptrack->size()!=0){
			calDistanceAndAngle(mCurPose,mCurPostion,ptrack->back().position,angle,distance);
			//bigAngle=abs(angle)>TRACK_BIG_ANGLE;
			//printf("catObstacleDistPoint angle:%f distance:%f,count:%d\n",angle,distance,count);
			if(distance > TRACK_MOVE_DIS_PONIT/*TRACK_LOOK_DIS_PONIT*/||ptrack->size()==1){
				PLOG_ERROR(NAV_PATH_NODE_TAG,"-- catObstacleDistPoint B.(angle:%f,distance:%f,count:%f,remain:%d)\r\n",
				angle,distance,count,ptrack->size());

				PLOG_ERROR(NAV_PATH_NODE_TAG,"-- mCurPostion(%f,%f) B(%f,%f)\r\n",
				mCurPostion.x(),mCurPostion.y(),ptrack->back().position.x(),ptrack->back().position.y());
				mLastObsDistance = distance;
				break;
			}else{
				ptrack->pop();
			}
			count++;
		}
		if(distance < TRACK_MOVE_DIS_PONIT/*TRACK_LOOK_DIS_PONIT*/ && ptrack->size()==1){
			ptrack->pop();
			PLOG_ERROR(NAV_PATH_NODE_TAG,"-- catObstacleDistPoint B error!distance:%f,size(%d) !!!!!\r\n",
						distance, ptrack->size());
			return false;
		} 

	}
	
	bool catObstacleDistPoint(double &angle){
		shared_ptr<TrackList> ObstracksList = mTrace.getTrackList();

	//rmwei
	
	double distanceAx1 = 0;
	 

	Eigen::Vector3d CurPostionX=Eigen::Vector3d(mCurPostion.x()+TRACK_LOOK_DIS_PONIT,mCurPostion.y(),0.0);
	calDistanceAndAngle(mCurPose,mCurPostion,CurPostionX,mObsStartAngleAX,distanceAx1);

#if 1
        if(ObstracksList->size()==0){
            return false;
        }

        double distance = 0;
        //double angle;
        bool bigAngle;
        int count = 0;
        double angleXB =0;
        
        while(ObstracksList->size()!=0){
            calDistanceAndAngle(mCurPose,mCurPostion,ObstracksList->back().position,angle,distance);
            //bigAngle=abs(angle)>TRACK_BIG_ANGLE;
            //printf("catObstacleDistPoint angle:%f distance:%f,count:%d\n",angle,distance,count);
            if(distance > TRACK_LOOK_DIS_PONIT||ObstracksList->size()==1){
            	PLOG_ERROR(NAV_PATH_NODE_TAG,"@@@@ catObstacleDistPoint B.(angle:%f,distance:%f,count:%f,remain:%d)\r\n",
            	angle,distance,count,ObstracksList->size());
            	
            	PLOG_ERROR(NAV_PATH_NODE_TAG,"@@@@ mCurPostion(%f,%f) B(%f,%f)\r\n",
            	mCurPostion.x(),mCurPostion.y(),ObstracksList->back().position.x(),ObstracksList->back().position.y());
            	mLastObsDistance = distance;
            	int direction = angle > 0 ? TRACK_DIRECTION_RIGHT:TRACK_DIRECTION_LEFT;
                double angleAB = angle;
                double angleAX = 0;
                double distanceAx = 0;
                Eigen::Vector3d CurPostionX=Eigen::Vector3d(mCurPostion.x()+TRACK_LOOK_DIS_PONIT,mCurPostion.y(),0.0);
                calDistanceAndAngle(mCurPose,mCurPostion,CurPostionX,angleAX,distanceAx);
                angleXB = -(angleAX - angleAB);
            	//ObsMaskAdd(1, angleXB);
                break;
            }else{
                ObstracksList->pop();
            }
            count++;
        }
        
        if(distance < TRACK_LOOK_DIS_PONIT && ObstracksList->size()==1){
            ObstracksList->pop();
            PLOG_ERROR(NAV_PATH_NODE_TAG,"catObstacleDistPoint B error!distance:%f,size(%d) !!!!!\r\n",
            			distance, ObstracksList->size());
            return false;
        }        
#endif


#if 1
		double obsRAngle = 0,obsLAngle = 0;
		double obsDistanceR = 0,obsDistanceL = 0;
		double rollAngle = M_PI/2;

		if( M_PI/4 > abs(angleXB) ||  ( (M_PI/2< abs(angleXB)) && (M_PI*3/4 > abs(angleXB)) ) ){
			//R
			int direction = TRACK_DIRECTION_RIGHT;
			rollAngle = M_PI/2;
			printf("R-----------%d\r\n",__LINE__);
			ObsFindSuitableSpace(direction,rollAngle,obsRAngle,obsDistanceR);
			
			if( (TRACK_MOVE_DIS_PONIT <= obsDistanceR) &&(0 < abs(obsRAngle)) ){ // R error
				mDirection = TRACK_DIRECTION_RIGHT;
				mObsDirRight = true;
				mObsCurentStatus = E_OBS_DIRRIGHT;
				mObsDirLeft = false;
				mObsStartAngle = obsRAngle-direction*(rollAngle+TRACK_MOVE_ANGLE_OFFET);
				PLOG_ERROR(NAV_PATH_NODE_TAG,"mDirection right.");
				return true;
			}
			//sleep(5);
			//L
			//reset
			mAlgo.roll(-direction*(rollAngle+TRACK_MOVE_ANGLE_OFFET),TRACK_ROLL_SPEED);
			
			direction = TRACK_DIRECTION_LEFT;
			rollAngle = M_PI/2;
			printf("L-----------%d\r\n",__LINE__);
			ObsFindSuitableSpace(direction,rollAngle,obsLAngle,obsDistanceL);
			
			if( (TRACK_MOVE_DIS_PONIT <= obsDistanceL) &&(0 < abs(obsLAngle)) ){ // R error
						mDirection = TRACK_DIRECTION_LEFT;
						mObsDirRight = false;
						mObsDirLeft = true;
						mObsCurentStatus = E_OBS_DIRLEFT;
						mObsStartAngle = obsLAngle-direction*(rollAngle+TRACK_MOVE_ANGLE_OFFET);
						PLOG_ERROR(NAV_PATH_NODE_TAG,"mDirection left.");
						return true;
			}
			
			//reset
			mAlgo.roll(-direction*(rollAngle+TRACK_MOVE_ANGLE_OFFET),TRACK_ROLL_SPEED);
		} else{
			//L		

			int direction = TRACK_DIRECTION_LEFT;
			double rollAngle = M_PI/2;
			printf("L-----------%d\r\n",__LINE__);
			ObsFindSuitableSpace(direction,rollAngle,obsLAngle,obsDistanceL);
			
			if( (TRACK_MOVE_DIS_PONIT <= obsDistanceL) &&(0 < abs(obsLAngle)) ){ // R error
						mDirection = TRACK_DIRECTION_LEFT;
						mObsDirRight = false;
						mObsDirLeft = true;
						mObsCurentStatus = E_OBS_DIRLEFT;
						mObsStartAngle = obsLAngle-direction*(rollAngle+TRACK_MOVE_ANGLE_OFFET);
						PLOG_ERROR(NAV_PATH_NODE_TAG,"mDirection left.");
						return true;
			}
			
			//reset
			mAlgo.roll(-direction*(rollAngle+TRACK_MOVE_ANGLE_OFFET),TRACK_ROLL_SPEED);

			direction = TRACK_DIRECTION_RIGHT;
			rollAngle = M_PI/2;
			printf("R-----------%d\r\n",__LINE__);
			ObsFindSuitableSpace(direction,rollAngle,obsRAngle,obsDistanceR);
			
			if( (TRACK_MOVE_DIS_PONIT <= obsDistanceR) &&(0 < abs(obsRAngle)) ){ // R error
				mDirection = TRACK_DIRECTION_RIGHT;
				mObsDirRight = true;
				mObsDirLeft = false;
				mObsCurentStatus = E_OBS_DIRLEFT;
				mObsStartAngle = obsRAngle-direction*(rollAngle+TRACK_MOVE_ANGLE_OFFET);
				PLOG_ERROR(NAV_PATH_NODE_TAG,"mDirection right.");
				return true;
			}
			//sleep(5);
			//L
			//reset
			mAlgo.roll(-direction*(rollAngle+TRACK_MOVE_ANGLE_OFFET),TRACK_ROLL_SPEED);
		}

		//sleep(5);
		do{
			if( (0 == obsDistanceL)&& (0 < abs(obsRAngle))){ // L error
					mDirection = TRACK_DIRECTION_RIGHT;
					mObsDirRight = true;
					mObsDirLeft = false;
					mObsCurentStatus = E_OBS_DIRRIGHT;
					mObsStartAngle = obsRAngle;
					PLOG_ERROR(NAV_PATH_NODE_TAG,"mDirection right.");
					break;
			}
			if( (0 == obsDistanceR) &&(0 < abs(obsLAngle)) ){ // R error
					mDirection = TRACK_DIRECTION_LEFT;
					mObsDirRight = false;
					mObsDirLeft = true;
					mObsCurentStatus = E_OBS_DIRLEFT;
					mObsStartAngle = obsLAngle;
					PLOG_ERROR(NAV_PATH_NODE_TAG,"mDirection left.");
					break;
			}
			if( ((TRACK_MOVE_DIS_PONIT <= obsDistanceL)&&(TRACK_MOVE_DIS_PONIT <= obsDistanceR)) && 
				(abs(obsRAngle) < abs(obsLAngle))){
				mDirection = TRACK_DIRECTION_RIGHT;
				mObsDirRight = true;
				mObsDirLeft = false;
				mObsCurentStatus = E_OBS_DIRRIGHT;
				mObsStartAngle = obsRAngle;
				PLOG_ERROR(NAV_PATH_NODE_TAG,"mDirection right.");
				break;
			}
			if(((TRACK_MOVE_DIS_PONIT <= obsDistanceL)&&(TRACK_MOVE_DIS_PONIT <= obsDistanceR)) && 
				(abs(obsRAngle) > abs(obsLAngle))){
				mDirection = TRACK_DIRECTION_LEFT;
				mObsDirRight = false;
				mObsDirLeft = true;
				mObsCurentStatus = E_OBS_DIRLEFT;
				mObsStartAngle = obsLAngle;
				PLOG_ERROR(NAV_PATH_NODE_TAG,"mDirection left.");
				break;
			}
			if(((TRACK_MOVE_DIS_PONIT <= obsDistanceL)&&(TRACK_MOVE_DIS_PONIT <= obsDistanceR)) &&
			(abs(obsRAngle) == abs(obsLAngle)) ){
				//mDirection = angle > 0 ? TRACK_DIRECTION_RIGHT:TRACK_DIRECTION_LEFT;
				if(obsDistanceL <= obsDistanceR){
					mDirection = TRACK_DIRECTION_RIGHT;
					mObsDirRight = true;
					mObsDirLeft = false;
					mObsCurentStatus = E_OBS_DIRRIGHT;
					mObsStartAngle = obsRAngle;
					PLOG_ERROR(NAV_PATH_NODE_TAG,"mDirection right.");
				}else{
					mDirection = TRACK_DIRECTION_LEFT;
					mObsDirRight = false;
					mObsDirLeft = true;
					mObsCurentStatus = E_OBS_DIRLEFT;
					mObsStartAngle = obsLAngle;
					PLOG_ERROR(NAV_PATH_NODE_TAG,"mDirection left.");
				}
				break;
			}
			//else
			mDirection = angle > 0 ? TRACK_DIRECTION_RIGHT:TRACK_DIRECTION_LEFT;
			if(TRACK_DIRECTION_RIGHT == mDirection){
				mDirection = TRACK_DIRECTION_RIGHT;
				mObsDirRight = true;
				mObsDirLeft = false;
				mObsCurentStatus = E_OBS_DIRRIGHT;
				mObsStartAngle = 0.0;
				PLOG_ERROR(NAV_PATH_NODE_TAG,"mDirection right.");
			}else{
				mDirection = TRACK_DIRECTION_LEFT;
				mObsDirRight = false;
				mObsDirLeft = true;
				mObsCurentStatus = E_OBS_DIRLEFT;
				mObsStartAngle = 0.0;
				PLOG_ERROR(NAV_PATH_NODE_TAG,"mDirection left.");
			}
			
		}while(0);
		//mAlgo.roll(direction*(rollAngle+TRACK_MOVE_ANGLE_OFFET),2.0);
#endif

		if( mObsDirRight || mObsDirLeft ){
			return true;
		}else{
			return false;
		}
    }

	void setNewTrack(shared_ptr<TrackList> track,bool flag,int needRoll = 1){

		PLOG_ERROR(NAV_PATH_NODE_TAG,"==>setNewTrack,mObsCurrent.size:(%d),track.size(%d)",
			mObsCurrent.size(),track->size());
		ObsEchoCurStatus();
		double tangle =0;
		double tdistance = 0;
		mRepeat = 0;
		ObsFindNeedMovePoint(track);
		calDistanceAndAngle(mCurPose,mCurPostion,track->back().position,tangle,tdistance);
		PLOG_ERROR(NAV_PATH_NODE_TAG,"==>setNewTrack,tangle:%f,tdistance:%f",
			tangle,tdistance);

		if(0 < abs(tangle)){

			PLOG_ERROR(NAV_PATH_NODE_TAG,"==>mAlgo.roll:%f ,start.",
				tangle);
				if( 1== needRoll){
					mAlgo.roll(tangle,TRACK_ROLL_SPEED); 
				}
		}

		PLOG_ERROR(NAV_PATH_NODE_TAG,"setNewTrack..");
		mTrace.setTrackList(track);
		setPatrolStatus(flag); 
	}

	void playCalibAudioLoop()
	{ 
		while(mBImuPatrolCalib || mBCalibByAkaze){
        	string cmd = "aplay /var/roller_eye/devAudio/calib.wav";
        	system(cmd.c_str());
			printf("%s\r\n",cmd.c_str());
		}
		mPlayingCalibAudio = false;
	}

    int ObsMaskAddcount =0;
	int ObsMaskCheck(double angle,double Threshold = 0.04){
		int ret = 0;
		printf("mObsMask.size(%d)\r\n",mIsObsMask.size());
		if( 0 == mIsObsMask.size()){
			return 0;
		}
		float xOffet =0.0;
		float yOffet =0.0;
		float xdist = cos(angle)*TRACK_MOVE_DIS_PONIT;
		float ydist = sin(angle)*TRACK_MOVE_DIS_PONIT;
		
		if(TRACK_DIRECTION_BACKWARD == mOBsEndPointDirection){//backward
			xOffet = mOBsEndPointDirection*TRACK_MOVE_DIS_PONIT;
			yOffet = mOBsEndPointDirection*TRACK_MOVE_DIS_PONIT;
		}
		
		cv::Rect2f DistMask(mCurPostion.x()+xdist+xOffet, mCurPostion.y()+ydist+yOffet, TRACK_MOVE_DIS_PONIT, TRACK_MOVE_DIS_PONIT);
		PLOG_ERROR(NAV_PATH_NODE_TAG,"ObsMaskCheck angle:%d\r\n",angle*M_PI/180);
        PLOG_ERROR(NAV_PATH_NODE_TAG,"ObsMaskCheck CurPostion [%f,%f,%f,%f]\r\n",mCurPostion.x(),mCurPostion.y(),TRACK_MOVE_DIS_PONIT,TRACK_MOVE_DIS_PONIT);
		PLOG_ERROR(NAV_PATH_NODE_TAG,"ObsMaskCheck xdist:%f ydist:%f ObsMaskAdd:%d\r\n",xdist,ydist,ObsMaskAddcount);

		for (auto iter = mIsObsMask.cbegin(); iter != mIsObsMask.cend(); iter++){
			cv::Rect2f Mask = DistMask & *iter;
			//if( 0.04*pow(TRACK_MOVE_DIS_PONIT,2) < Mask.area()){
			if( Threshold*pow(TRACK_MOVE_DIS_PONIT,2) < Mask.area()){
				ret = -1;
				PLOG_ERROR(NAV_PATH_NODE_TAG,"angle:%d mask %d%%!!!!!!\r\n",(int)round(angle*180/M_PI),(int)round(100*Mask.area()/pow(TRACK_MOVE_DIS_PONIT,2)));
				break;
			}
		}
	    return ret;
	}

	int ObsMaskAdd(int flag,double angle){
        ObsMaskAddcount++;
		float xdist =0.0;
		float ydist  =0.0;
		
		float xOffet =0.0;
		float yOffet  =0.0;
		if(1== flag){
			xdist = cos(angle)*TRACK_MOVE_DIS_PONIT;
			ydist = sin(angle)*TRACK_MOVE_DIS_PONIT;
		}
		if(TRACK_DIRECTION_BACKWARD == mOBsEndPointDirection){
			xOffet = mOBsEndPointDirection*TRACK_MOVE_DIS_PONIT;
			yOffet = mOBsEndPointDirection*TRACK_MOVE_DIS_PONIT;
		}
		cv::Rect2f curMask(mCurPostion.x()+xdist+xOffet, mCurPostion.y()+ydist+yOffet, 
						TRACK_MOVE_DIS_PONIT, TRACK_MOVE_DIS_PONIT);

        PLOG_ERROR(NAV_PATH_NODE_TAG,"ObsMaskAdd angle:%f\r\n",angle*M_PI/180);
		PLOG_ERROR(NAV_PATH_NODE_TAG,"ObsMaskAdd [%f,%f,%f,%f],ObsMaskAdd:%d\r\n",
        mCurPostion.x()+xdist,mCurPostion.y()+ydist,mOBsEndPointDirection*TRACK_MOVE_DIS_PONIT,mOBsEndPointDirection*TRACK_MOVE_DIS_PONIT,ObsMaskAddcount);
		mIsObsMask.push_back(curMask);
        
	}

    int ObsGetDistance(double angle,double &distance)
    {
    	int repeat = 0,ret = -1;
    	if(0 < abs(angle)){
			mAlgo.roll(angle,TRACK_ROLL_SPEED);
    	}
    	distance = 0;
		mTofDistance=FLT_MAX;
		while(FLT_MAX == mTofDistance){
			if(20 < repeat++){
				break;
			}
			usleep(50*1000);
		}
		if(FLT_MAX != mTofDistance){
			distance = mTofDistance;
			ret = 0;
		}
		return ret;
    }


	int ObsMovceCeil(int direction,double angle){
//		double dist;
//		ObsGetDistance(direction*angle-mObsAngleOffet,dist);
//		mObsAngleOffet =direction*angle;
#if 1
		
		if( (-1 == mlastStatus) && (0 == ObsMaskCheck(-direction*angle)) ){

			double DistPoint;
			ObsGetDistance(direction*angle-mObsAngleOffet,DistPoint);
			mObsAngleOffet =direction*angle;
			printf("point %f angles:%f\r\n", angle*180/M_PI, mObsAngleOffet);
			if( ((TRACK_MOVE_DIS_PONIT) < DistPoint) ){
				if( M_PI/2 <= abs(angle) ){
					printf("=========ObsMaskAdd 0.\r\n");
					ObsMaskAdd(0, -direction*angle);
				}
				
				if(mObsDistance > TRACK_LOOK_DIS_PONIT){
					mAlgo.move(0.0,TRACK_MOVE_DIS_PONIT+0.1,TRACK_MOVE_SPEED);
				}else{
					mAlgo.move(0.0,mObsDistance,TRACK_MOVE_SPEED);
				}
				
				printf("point %f -->:%f,dist:%f\r\n",angle*180/M_PI,mObsAngleOffet,DistPoint);
				mlastStatus = 0;
			}else{
				ObsMaskAdd(1, -direction*angle);
			}			

		}
#endif
	}
	void ObsDirReset(){
		if(abs(mObsAngleOffet) > 0.0){
			mAlgo.roll(0-mObsAngleOffet,TRACK_ROLL_SPEED);
			printf("point reset angles:%f\r\n", -mObsAngleOffet);
			mObsAngleOffet = 0;
		}
	}

	void ObsSetTrackOffet(){
		ObsPathChanelSync();
		shared_ptr<TrackList> ObstracksList = mTrace.getTrackList();
		while(0 < mTrackListIndex){
			if(0 < ObstracksList->size()){
				ObstracksList->pop();
			}
			mTrackListIndex--;
		}
		auto iter = ObstracksList->TrackPointList().rbegin();		

		//iter+=mTrackListIndex;
		Eigen::Vector3d  mObsPostionOffet = mCurPostion - iter->position;
		PLOG_ERROR(NAV_PATH_NODE_TAG,"mCurPostion:(%f,%f,%f)\r\n", mCurPostion.x(),mCurPostion.y(),mCurPostion.z());
		PLOG_ERROR(NAV_PATH_NODE_TAG,"iter->position:(%f,%f,%f)\r\n", iter->position.x(),iter->position.y(),iter->position.z());
		PLOG_ERROR(NAV_PATH_NODE_TAG,"mObsPostionOffet:(%f,%f,%f)\r\n", mObsPostionOffet.x(),mObsPostionOffet.y(),mObsPostionOffet.z());

		mObsTrackOffet.clear();
		mObsLastTracks.clear();
		for (auto iter = ObstracksList->TrackPointList().begin(); iter != ObstracksList->TrackPointList().end(); iter++){
            Eigen::Vector3d pos=iter->position + mObsPostionOffet;
            Eigen::Quaterniond pose =iter->orientation;
            TrackPoint point(ros::Time::now().toNSec(),pos,pose,0,0,0);
			mObsTrackOffet.push(point);
			
		}

	}
	
	int ObsCheckDistStatus(){
		int status = 0;
		if(0 < abs(mObsStartAngle)) {
			return status;
		}
		//mObsTrackOffet.clear();
		catTwoPointDist(mObsAngle,mObsDistance);
//		if( ((TRACK_MOVE_ANGLE_OFFET+M_PI/2-10*M_PI/180) < abs(mObsAngle)) &&
//		((TRACK_MOVE_ANGLE_OFFET+M_PI/2+10*M_PI/180) > abs(mObsAngle)) ){
//			mObsMoveAngle++;
//		}else{
//			mObsMoveAngle = 0;
//		}
		double DistPoint;
		//ObsGetDistance(0,DistPoint);
		//mObsStartAngleAX
		//ObsGetDistance(mObsAngle,DistPoint);

//		PLOG_ERROR(NAV_PATH_NODE_TAG,"@@@@@@ObsCheckDistStatus mObsMoveAngle(%d,%f,%f)\r\n",mObsMoveAngle,mObsDistance,DistPoint);

		double angleAX = 0;
		double distanceAx = 0;
		Eigen::Vector3d CurPostionX=Eigen::Vector3d(mCurPostion.x()+TRACK_LOOK_DIS_PONIT,mCurPostion.y(),0.0);
		calDistanceAndAngle(mCurPose,mCurPostion,CurPostionX,angleAX,distanceAx);
		PLOG_ERROR(NAV_PATH_NODE_TAG,"@@@@@@angleAX:%f,distanceAx:%f,dx:%f",
		angleAX,distanceAx,abs(mObsDistance*cos(M_PI -(mObsAngle - angleAX) )));
		double dAngle = angleAX -mObsStartAngleAX;
		ObsGetDistance(dAngle,DistPoint);
		//PLOG_ERROR(NAV_PATH_NODE_TAG,"@@@@@@ObsCheckDistStatus mObsMoveAngle(%d,%f,%f)\r\n",mObsMoveAngle,mObsDistance,DistPoint);
		
		
		double ChanelDistance = abs(mObsDistance*cos(M_PI -(mObsAngle - angleAX) ));

//		CurPostionX=Eigen::Vector3d(mCurPostion.x()+TRACK_LOOK_DIS_PONIT,mCurPostion.y(),0.0);
//		calDistanceAndAngle(mCurPose,mCurPostion,CurPostionX,angleAX,distanceAx);
//		if(-1 == ObsMaskCheck(0,0.5)){
//			return status;
//		}
		shared_ptr<TrackList> ObstracksList = mTrace.getTrackList();
		
		Eigen::Vector3d  mObsPostionOffet = mCurPostion - ObstracksList->back().position;
		PLOG_ERROR(NAV_PATH_NODE_TAG,"==>DistPoint:%f,mObsPostionOffet:%f",
		DistPoint,mObsPostionOffet);

		if( TRACK_LOOK_DIS_PONIT < abs(mObsPostionOffet.y()) ){
			return status;
		}
		

		//catTwoPointDist(mObsAngle,mObsDistance);
		//sleep(5);
		if(-1 == mObsDistance){
			PLOG_ERROR(NAV_PATH_NODE_TAG,"@@@@@@catTwoPointDist error.(%f)\r\n",mObsDistance);
		}


		if( /*(1 < mObsMoveAngle)(TRACK_PATH_ANGLE_OFFSET > abs(mObsFurthestPointAngle)) &&*/
			((TRACK_MOVE_DIS_PONIT) < DistPoint) && ( (TRACK_PATH_MOVE_OFFSET+0.1) > ChanelDistance/*mObsDistance*/) ){
			PLOG_ERROR(NAV_PATH_NODE_TAG,"@@@@@@@@@@@@@@@#######(%f)\r\n",mObsDistance);
			ObsSetTrackOffet();
			status	= -1;
			return status;
		}	
		
			if( ( (TRACK_MOVE_DIS_PONIT)<DistPoint) &&(0 <= mObsDistance) && (TRACK_MOVE_DIS_PONIT+0.1 > ChanelDistance/*mObsDistance*/)
			  ){

			if(mObsDistance < DistPoint){
				PLOG_ERROR(NAV_PATH_NODE_TAG,"@@@@@@@@@@@@@@@@@@@@@@@@@@@@(%f,%f)DistPoint\r\n",mObsDistance,DistPoint);
				status	= -1;
				return status;
			}
		}	

		return status;
	}
	
	int ObsMoveOnce(int direction){
		printf("\r\n=========ObsMoveOnce start(%d).=========\r\n",mDirection);
		catTwoPointDist(mObsAngle,mLastObsDistance);
		cv::Rect2f curMask(mCurPostion.x(), mCurPostion.y(), TRACK_MOVE_DIS_PONIT, TRACK_MOVE_DIS_PONIT);

		shared_ptr<TrackList> ObstracksList = mTrace.getTrackList();
		
		auto iter = ObstracksList->TrackPointList().rbegin();
		iter+=mTrackListIndex;
		printf("Dist Ponit (%f,%f),(Angle:%f,mObsDistance:%f)\r\n",iter->position.x(),iter->position.y(),
								mObsAngle*180/M_PI,mLastObsDistance);

								
		mlastStatus = -1;
		mObsAngleOffet = 0;
		mRepeat++;


		double obsAngle = 0;
		double obsDistance = 0;
		double rollAngle = M_PI;
		if( (0 < abs(mObsStartAngle)) || ObsFindSuitableSpace(direction,rollAngle,obsAngle,obsDistance)){
			if(0 < abs(mObsStartAngle)) {
				mAlgo.roll(mObsStartAngle,TRACK_ROLL_SPEED);
				mObsStartAngle = 0;
			}else{
				mAlgo.roll((obsAngle-direction*(rollAngle+TRACK_MOVE_ANGLE_OFFET)),TRACK_ROLL_SPEED);
				//

			}
			
			if( (M_PI/2+TRACK_MOVE_ANGLE_OFFET) <= abs(obsAngle) ){
				printf("=========ObsMaskAdd 0.\r\n");
				ObsMaskAdd(0, -direction*obsAngle);
				//mRepeat++;
			}
			printf("point %f -->:%f,dist:%f,mLastObsDistance:%f\r\n",
			obsAngle*180/M_PI,obsAngle,obsDistance,mLastObsDistance);
			if(mLastObsDistance > TRACK_MOVE_DIS_PONIT){
				double movedist =0;
				double tdist = abs(cos(obsAngle)*mLastObsDistance);
				if(tdist < TRACK_MOVE_DIS_PONIT ){
					movedist = TRACK_MOVE_DIS_PONIT+0.1;
				}else{
					movedist = tdist +0.1;
				}
				PLOG_ERROR(NAV_PATH_NODE_TAG,"##movedist:%f.",movedist);
//				mAlgo.move(0.0,movedist,TRACK_MOVE_SPEED);
				mAlgo.move(0.0,TRACK_MOVE_DIS_PONIT+0.1,TRACK_MOVE_SPEED);
			}else{
				mAlgo.move(0.0,mLastObsDistance,TRACK_MOVE_SPEED);
				//mAlgo.move(0.0,TRACK_MOVE_DIS_PONIT+0.1,TRACK_MOVE_SPEED);
			}

			mlastStatus = 0;	
		}else{
			ObsMaskAdd(0, -direction*obsAngle);
			//mRepeat++;
		}
	
		if(0 == mlastStatus){
			mObsMask.push_back(curMask);
		}
		printf("=========ObsMoveOnce end.=========\r\n");
		//mlastStatus = status;
		return mlastStatus;
	}

	void ObsPathChanelSync(){
	
		PLOG_ERROR(NAV_PATH_NODE_TAG,"Sync! mObsTrackOffet size:%d,mObsLastTracks size:%d,mTrace.size(%d),mObstracks.size(%d),mObsCurrent.size(%d)",
		mObsTrackOffet.size(),mObsLastTracks.size(),mTrace.size(),mObstracks->size(),mObsCurrent.size());

		if(0 < mTrackListIndex){
			mTrace.clipTrace(mTrackListIndex);
			mTrackListIndex =0;
		}
		if(0 < mObsTrackOffet.size()){
			if(mObsTrackOffet.size() > mTrace.size() ){
				int offetCount = mObsTrackOffet.size() - mTrace.size();
				while(0 < offetCount){
					if(0 < mObsTrackOffet.size()){
						mObsTrackOffet.pop();
					}
					offetCount--;
				}				
			}
		}

	}
	
	bool avoidObstacleUpdateStaus(){
		if( 0 == misAvoidObs){
			PLOG_ERROR(NAV_PATH_NODE_TAG,"New OBS!");
			mObsStartAngle = 0;
			double angle;
			mTrackListIndex = 0;
			ObsPathChanelSync();
			mObsTrackOffet.clear();
			
			catObsEndPointAngle();
			PLOG_ERROR(NAV_PATH_NODE_TAG,"mOBsEndPointAngle:%f,mOBsEndPointDistance%f",mOBsEndPointAngle,mOBsEndPointDistance);
//			if( (TRACK_DIRECTION_BACKWARD == mOBsEndPointDirection) && (1.5 > mOBsEndPointDistance) ){
//				PLOG_ERROR(NAV_PATH_NODE_TAG,"Near the charging pile.");
//				setPatrolStatus(false);
//				return false;
//			}
			if(!catObstacleDistPoint(angle)){
				// setPatrolStatus(false);
				PLOG_ERROR(NAV_PATH_NODE_TAG,"Near the end.");
				return false;
			}
			
			if( mTrace.size()<=20 && !mObjDetect){   //40cm
				mDetectedPile = false;  
				mObjDetect=mGlobal.subscribe("CoreNode/chargingPile",1,&NavigatePath::onObjdetect,this);
			}
 
			mObsStartPose = mCurPose;
			mObsStartPostion = mCurPostion;
			//mDirection = angle > 0 ? -1:1;

			
			mObsReturn = false;

			mObsMask.clear();
			//mIsObsMask.clear();
			misAvoidObs = 1;
			misObsBackHome =0;
			mRepeat = 0;
			mTwoside = 0;
			mObsHome.clear();
			if(TRACK_DIRECTION_BACKWARD == mOBsEndPointDirection){
				shared_ptr<TrackList> ObstracksList = mTrace.getTrackList();
				mObsHome = *ObstracksList;
				mCurrent = *ObstracksList;
				*mObstracks =*ObstracksList;
				if(0 < mObstracks->size()){
					if(mObstracks->size() > mTrace.size() ){
						int offetCount = mObstracks->size() - mTrace.size();
						while(0 < offetCount){
							if(0 < mObstracks->size()){
								mObstracks->pop();
							}
							offetCount--;
						}				
					}
				}

				PLOG_ERROR(NAV_PATH_NODE_TAG,"TRACK_DIRECTION_BACKWARD.");
			}else{
				mObsHome = mCurrent;
				PLOG_ERROR(NAV_PATH_NODE_TAG,"TRACK_DIRECTION_FORWARD.");
			}			
			ObsEchoCurStatus();
		}else{
			mRepeat = 0;
			printf("Old OBS!\r\n");
		}
		return true;
	}

	int ObsTryOtherSide(int needRoll = 1){
		mObsReturn = false;
		PLOG_ERROR(NAV_PATH_NODE_TAG,"====>ObsTryOtherSide.");
		//PLOG_ERROR(NAV_PATH_NODE_TAG,"ObsTryOtherSide!");
		if( (!mObsDirRight) || (!mObsDirLeft) ){
			
			if(!mObsDirRight){
				mDirection = TRACK_DIRECTION_RIGHT ;
				//printf("turn Direction right.\r\n");
				PLOG_ERROR(NAV_PATH_NODE_TAG,"====>turn Direction right.");
				mObsDirRight =true;
				mObsCurentStatus = E_OBS_DIRRIGHT;
				mRepeat = 0;
				setNewTrack(mObstracks, true,needRoll);
			}
			if(!mObsDirLeft){
				mDirection = TRACK_DIRECTION_LEFT;
				//printf("turn Direction left.\r\n");
				PLOG_ERROR(NAV_PATH_NODE_TAG,"====>turn Direction left.");
				mObsDirLeft = true;
				mObsCurentStatus = E_OBS_DIRLEFT;
				mRepeat = 0;
				setNewTrack(mObstracks, true,needRoll);
			}
		}else{
			//printf("back home 0 !!!!!!!!!!!!!\r\n");
			PLOG_ERROR(NAV_PATH_NODE_TAG,"====>ObsTryOtherSide back home.");
//			auto track=make_shared<TrackList>();
//			*track = mObsHome;
//			setNewTrack(track, true);
			mIsObsMask.clear();
			ObsBackHome(needRoll);
		}
	}

	int ObsBackHome(int needRoll = 1){
		PLOG_ERROR(NAV_PATH_NODE_TAG,"====>ObsBackHome.");
		if(0 == misObsBackHome){
			auto track=make_shared<TrackList>();
			*track = mObsHome;
			mObsCurentStatus = E_OBS_BACKHOME;
			setNewTrack(track, true,needRoll);
			//sleep(5);
	//		mObstracks = track;//rmwei
	//		double angle;
	//		catObstacleDistPoint(angle);
	//		if( 0 < abs(angle) ){
	//			mAlgo.roll(angle,2.0);
	//		}

			//sleep(5);
			misObsBackHome = 1;
			mDirection = -mDirection;
		}
		//misAvoidObs = 0;
	}
	
	int ObsCheckCurStatus(int status){
		if( /*(0 == status)&&*/( (E_OBS_BACKHOME == mObsCurentStatus) )  ){
			PLOG_ERROR(NAV_PATH_NODE_TAG,"====>avoidObstacle2 ok.");
			mIsObsMask.clear();
			
			ObsPathChanelSync();
			shared_ptr<TrackList> ObstracksList = mTrace.getTrackList();
			*mObstracks =  *ObstracksList;

			if(0 < mObsTrackOffet.size()){
				auto track=make_shared<TrackList>();
				*track = mObsTrackOffet;
				setNewTrack(track, true);
				mObsTrackOffet.clear();
				//setNewTrack(mObstracks, true);
				PLOG_ERROR(NAV_PATH_NODE_TAG,"====>mObsTrackOffet ok.");
			}else{
				setNewTrack(mObstracks, true);
			}
			misAvoidObs = 0;
			mObsCurentStatus = 0;
			return 0;
		}	

	   if( ( (E_OBS_DIRLEFT == mObsCurentStatus) ||(E_OBS_DIRRIGHT == mObsCurentStatus) ) &&
			(0 == status)  ){ 
			PLOG_ERROR(NAV_PATH_NODE_TAG,"====>avoidObstacle2 ok.");
			mIsObsMask.clear();
			mObsCurentStatus = 0;
			
			ObsPathChanelSync();
			shared_ptr<TrackList> ObstracksList = mTrace.getTrackList();
			*mObstracks =  *ObstracksList;


			
			if(0 < mObsTrackOffet.size()){

				auto track=make_shared<TrackList>();
				*track = mObsTrackOffet;
				setNewTrack(track, true);
				mObsTrackOffet.clear();
				PLOG_ERROR(NAV_PATH_NODE_TAG,"====>mObsTrackOffet ok.");
			}else{

				setNewTrack(mObstracks, true);
			}
			misAvoidObs = 0;
			
			return 0;
		}
		
		if(( (E_OBS_DIRLEFT == mObsCurentStatus) ||(E_OBS_DIRRIGHT == mObsCurentStatus) ) && 
			(-1 == status)){
			printf("====>Return to the origin and try from the other side.#\r\n\r\n");
			auto track=make_shared<TrackList>();
			*track = mObsCurrent;
			//mObsCurrent.clear();
			mObsCurentStatus = E_OBS_RETURN;
			setNewTrack(track, true);
			mObsReturn = true;
			mDirection = -mDirection;
			mIsObsMask.clear();
			return 0;
		}
		
		if( (E_OBS_RETURN == mObsCurentStatus) && 
			(0 == status)  ){

			PLOG_ERROR(NAV_PATH_NODE_TAG,"====>Return start point ok.(%d)#",mObstracks->size());
			ObsTryOtherSide();
			mIsObsMask.clear();
			mTwoside++;
			return 0;
		}

		if((E_OBS_RETURN == mObsCurentStatus) && (-1 == status) ){
			PLOG_ERROR(NAV_PATH_NODE_TAG,"====>Return start point error!and ObsBackHome.");		

			mIsObsMask.clear();
			ObsBackHome();
			return 0;
			//sleep(10);
		}

		return 0;
	}

	int ObsMoveOnceThread(){
		int status = -1;
		while( misAvoidObs ){
			if(0 != ObsCheckDistStatus()){
				status  = 0;
				break;
			}
			if( true == mDetectedPile){
				status	= 0;
				break;
			}
			if(mRepeat >= TRACK_MOVE_REPEAT){
				break;
			}
			double LastPointDistance = mObsFurthestPointDistance;
			if(0 != ObsMoveOnce(mDirection)){
				break;
			}
			catTwoPointDist(mObsAngle,mObsDistance);
			printf("mObsDistance:%f, mLastObsDistance:%f,mRepeat:%d\r\n",
			mObsDistance , mLastObsDistance,mRepeat);

			if( mObsFurthestPointDistance > LastPointDistance ){
				//mRepeat++;
			}
			if(mIsCancel){
				break;
			}
		}
		return status;
	}

	int ObsFindSuitableSpace(int direction,double rollAngle,double &obsAngle,double &obsDistance){
		auto pdict = make_shared<DistanceMap>();
		catTwoPointDist(mObsAngle,mObsDistance);
		double  resetAngle = mObsAngle - direction*TRACK_MOVE_ANGLE_OFFET;
		mAlgo.roll(resetAngle,TRACK_ROLL_SPEED);
		double dectAngle = direction*(rollAngle+2*TRACK_MOVE_ANGLE_OFFET);
        PLOG_ERROR(NAV_PATH_NODE_TAG,"ObsFindSuitableSpace dectAngle:%f !", dectAngle*M_PI/180);
		mAlgo.getDistance(dectAngle,pdict);
		printf("pdict->size(%d)\r\n",pdict->size());
		obsAngle = 0.0;
		obsDistance = 0.0;
		bool gotAngle = false;
		for (auto iter = pdict->begin(); iter != pdict->end(); iter++){
			double	angle= iter->first;
			double distance = iter->second;
			if( (TRACK_MOVE_DIS_PONIT) < distance ){
				auto iterLocal = iter;
				double distAngle = angle + direction*2*TRACK_MOVE_ANGLE_OFFET;
				
				int TrueCount = 0;
				int TotalCount = 0;
				
				for(;iterLocal !=  pdict->end();iterLocal++){
					double	dangle = iterLocal->first;
					double ddistance = iterLocal->second;
					PLOG_DEBUG(NAV_PATH_NODE_TAG,"	#dangle:%f,ddistance:%f !", dangle,ddistance);
					if(abs(distAngle) < abs(dangle) )break;
					TotalCount++;
					if((TRACK_MOVE_DIS_PONIT) <= ddistance) {
						TrueCount++;
					}else{
						printf("error,dangle:%f,ddistance:%f !\r\n", dangle,ddistance);
                        iter += TotalCount;
						break;
					}
				}
				printf("TotalCount:%d,TrueCount:%d\r\n", TotalCount , TrueCount);
				auto iterTmp = iter;
				iterTmp += int(TotalCount/2);
                //rmwei
                double angleBC = iterTmp->first;
                double angleAX = 0;
                double distanceAx =0;
                Eigen::Vector3d CurPostionX=Eigen::Vector3d(mCurPostion.x()+TRACK_LOOK_DIS_PONIT,mCurPostion.y(),0.0);
                calDistanceAndAngle(mCurPose,mCurPostion,CurPostionX,angleAX,distanceAx);  
                double angleAB = 0;
                double distanceAB =0;
                catTwoPointDist(angleAB,distanceAB);
                PLOG_ERROR(NAV_PATH_NODE_TAG,"angleAB:%f,distanceAB:%f\r\n",
                                            angleAB,distanceAB); 
                double  angleXC =   angleAB +  angleBC -  angleAX; 
                PLOG_ERROR(NAV_PATH_NODE_TAG,"angleXC:%f,angleAB:%f,angleBC:%f,angleAX:%f\r\n",
                                            angleXC,angleAB,angleBC,angleAX);    
				if( ( 0< TotalCount) && (TotalCount == TrueCount) &&
					(0 == ObsMaskCheck(angleXC))/*(0 == ObsMaskCheck(direction*iterTmp->first))*/ ){
					PLOG_ERROR(NAV_PATH_NODE_TAG,"==>got one:angle:%f,distAngle:%f\r\n",iterTmp->first,iterTmp->second);
					gotAngle = true;
					obsAngle = iterTmp->first;
					obsDistance = iterTmp->second;
				}
				if(gotAngle) break;
			}
			if(gotAngle) break;	
		}
		if(!gotAngle){
			PLOG_ERROR(NAV_PATH_NODE_TAG,"ObsFindSuitableSpace error!"); 	
		}
		return gotAngle;
	}
	void ObsBackHomeThread(){
		misObsBackHome =0;
		misAvoidObs =0;
		mDoBacking = true;
		ObsEchoCurStatus();
		setPatrolStatus(false);
		stop();
		mObsCurentStatus = E_OBS_BACKUP;
		auto th=std::thread([this](){  
			mBackUp.begin(mGlobal);
		});
		th.detach();
	}

	void ObsEchoCurStatus(){
		switch(mObsCurentStatus){
			case E_OBS_DIRLEFT:
				PLOG_ERROR(NAV_PATH_NODE_TAG,"@@> E_OBS_DIRLEFT.%s",(TRACK_DIRECTION_FORWARD == mOBsEndPointDirection) ? "FORWARD" : "BACKWARD");
				break;
			case E_OBS_DIRRIGHT:
				PLOG_ERROR(NAV_PATH_NODE_TAG,"@@> E_OBS_DIRRIGHT.%s",(TRACK_DIRECTION_FORWARD == mOBsEndPointDirection) ? "FORWARD" : "BACKWARD");
				break;
			case E_OBS_RETURN:
				PLOG_ERROR(NAV_PATH_NODE_TAG,"@@> E_OBS_RETURN.%s",(TRACK_DIRECTION_FORWARD == mOBsEndPointDirection) ? "FORWARD" : "BACKWARD");
				break;
			case E_OBS_BACKHOME:
				PLOG_ERROR(NAV_PATH_NODE_TAG,"@@> E_OBS_BACKHOME.%s",(TRACK_DIRECTION_FORWARD == mOBsEndPointDirection) ? "FORWARD" : "BACKWARD");
				break;
			case E_OBS_BACKUP:
				PLOG_ERROR(NAV_PATH_NODE_TAG,"@@> E_OBS_BACKUP.%s",(TRACK_DIRECTION_FORWARD == mOBsEndPointDirection) ? "FORWARD" : "BACKWARD");
				break;
			default:
				PLOG_ERROR(NAV_PATH_NODE_TAG,"@@> E_OBS_RESET.%s",(TRACK_DIRECTION_FORWARD == mOBsEndPointDirection) ? "FORWARD" : "BACKWARD");
				break;
		}
	}

	void avoidObstacle2(){
		//Turn right 

		mObsStatus = true;
		mObsCurrent.clear();

		if( avoidObstacleUpdateStaus()){
			int status = ObsMoveOnceThread();
			ObsCheckCurStatus(status);
		}else{
			mTrace.clear();
			misAvoidObs = 0;
			return;
		}
		mObsStatus = false;
		
		if(mDetectedPile){
			ObsBackHomeThread();
		}

	}
  
 
    bool detectPileInSitu()      
    {
        AlgoOBjPos pos; 
        const string HOME="home";

        bool ret = false;
        int timeout = 5000;
        for (int i=0; i<2; i++){
            if(mAlgo.waitObj2(HOME,timeout,pos)==0){
                    ret = true;
                    break;
            }
            if(i>0 || mAlgo.roll(M_PI,1.0)<0){
                    break;
            }
            timeout -= 2000;
        }
        
        if (!mDoBacking){
            ret = false;
        }
        
        return ret;
    }
 
 
    void courseReversal()
    {
        if (detectPileInSitu()){
            std_msgs::Int32 status;
            status.data = 1;
            bool bRet = save_path(mPatrolingPathName);
            status.data = bRet?0:1;
            mNavTraceDonePub.publish(status);         
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
                    misAvoidObs = 0;
                    mObsStartAngle = 0;
                    mIsObsMask.clear();
                    mObstracks = track;
                    setPatrolStatus(true);
                    setPatrolStatus(true);
                }      
            }
        } 
        mDetectingPiling = false;
    }
    
    void batteryStatusCallback(const statusConstPtr &s)
    {   
        mCurBatPercentage=s->status[1];
        if (mPrevBatPercentage>mMinBat 
           && mCurBatPercentage<mMinBat){
            mLowBatFlag = true;
        }else if (mCurBatPercentage>mMinBat){
            mLowBatFlag = false;
        }

        static bool bBack = false;
        if(mLowBatFlag && 
            s->status[0] != status::BATTERY_CHARGING ){
            if (mIsPatroling && !bBack){
                if (mTrace.getTracePercent()<0.5){
                    auto track=make_shared<TrackList>();
                    *track=mCurrent;
                    mTrace.setTrackList(track);
                    bBack = true;
                }
            }
        }else{
            bBack = false;
        }
        mPrevBatPercentage = mCurBatPercentage;
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
            PLOG_INFO(NAV_PATH_NODE_TAG,"First pair of IMU and magnetometer messages received.");
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
    ros::ServiceClient  mDetectRecordStatusClient;
    ros::ServiceClient  mImuPatrolCalibClient;
    ros::ServiceClient  mImuPatrolCalibStatusClient;
    ros::ServiceClient  mSaveTmpPicClient;
    ros::ServiceClient  mGetDiffAngleClient;
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
    ros::Subscriber mOdomRelative;
    ros::Subscriber mTof;
    ros::Publisher mPose;
    ros::Publisher mCmdVel;
    ros::Publisher mNavTraceDonePub;
    atomic<bool> mDoingMagCalibra;
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
	//rmwei voidObstacle
	bool mBImuPatrolCalib = false;
	bool mBCalibByAkaze = false;
	bool mObsStatus = false;
	bool mPlayingCalibAudio = false;
	atomic<int> mObsdistCnt;
	TrackList mObsCurrent;
	TrackList mObsHome;
	
	TrackList mObsTrackOffet;
	TrackList mObsLastTracks;
	shared_ptr<TrackList> mObstracks;
	std::vector<cv::Rect2f> mObsMask;
	std::vector<cv::Rect2f> mIsObsMask;
	
    Eigen::Quaterniond mObsStartPose;
    Eigen::Vector3d  mObsStartPostion;
    bool mObsDirLeft = false;
    bool mObsDirRight = false;
    int mDirection = 0;
    bool mObsReturn = false;

	double mObsStartAngle = 0.0;
	double mObsStartAngleAX = 0.0;
	double mObsAngle = 0.0;
	double mObsDistance = 0.0;
	double mTofDistance = 0.0;
	double mLastObsDistance = 0.0;

	double mObsFurthestPointAngle = 0.0;
	double mObsFurthestPointDistance = 0.0;
	int mObsMoveAngle = 0;
	
	Eigen::Vector3d  mObsPostionOffet;
	
	float  mResetPointDist = 0.0;
	int    mRepeat = 0;

	int misAvoidObs = 0;
	int misObsBackHome = 0;
	int mlastStatus = -1;
	int mObsCurentStatus = 0;
	
	int mTwoside = 0;
	float mObsAngleOffet = 0.0;
	int mTrackListIndex = 0;
	int mObsRetry = 0;
	ros::Subscriber mObsTof;

	double mOBsEndPointAngle = 0.0;
	double mOBsEndPointDistance = 0.0;
	double mOBsEndPointDirection = 1;
	//end rmwei
    TrackList mCurrent;
    TrackTrace mTrace;
    Eigen::Quaterniond mCurPose;
    Eigen::Vector3d  mCurPostion;
    string mPatrolingPathName;
    bool mIsTracing;                                //is move by path

    bool mPathPlanning=false;
    double mLastTraceStamp;

    AlgoUtils mAlgo;
    atomic<bool> mAdjusting;

    StatusPublisher mPub;


    DistanceCloud mDistCloud;

    BackingUpHelper mBackUp;
    ros::Publisher mSysEvtPub;
    ros::Publisher mPatrolStatusPub;

    ros::Timer mTimer;

    DeviceDefaultConfig mCfg;

    int mMinBat;
    ros::Subscriber mBatteryStatus;
    ros::Subscriber mObjDetect;
    bool mBUseVio;    

    bool mClipData;
    bool mIsPatroling;                                //patrol state
    bool mDetectingPiling;                          //detect chargepile when return by one key


    bool mLowBatFlag;
    bool mIsStartFromOut;	
    bool mDetectedPile;

    int mPrevBatPercentage;

    int mCurBatPercentage;
    int mMaxPathSize;

    int mTofAvgCnt;
    float mAvgTof;

    bool mIsCancel;
    bool mDoBacking;
	atomic<int> mDetectObstacleCnt;
    float OBSTACLE_MIN_DISTANCE;
	int mTraceCounter;
	double mSumW;
};
int main(int argc, char **argv)
{
  ros::init(argc, argv, "NavPathNode");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,NAV_NODE_DEBUG_LEVEL);
  NavigatePath nav;
  ros::spin();
  return 0;
}
