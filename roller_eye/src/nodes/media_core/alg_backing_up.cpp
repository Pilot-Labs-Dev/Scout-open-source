#include<unistd.h>
#include<cmath>
#include<numeric>
#include<vector>
#include"roller_eye/status.h"
#include"roller_eye/plog_plus.h"
#include"roller_eye/plt_assert.h"
#include"roller_eye/graphic_utils.h"
#include "roller_eye/motor.h"
#include"alg_backing_up.h"
#include<opencv2/core/eigen.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "roller_eye/adjust_ligth.h"
#include "roller_eye/night_get.h"
#include"roller_eye/nav_cancel.h"
#include "roller_eye/system_define.h"
#include"roller_eye/plt_config.h"
#include "roller_eye/param_utils.h"
#include <tf2/LinearMath/Vector3.h>

#define BACKING_UP_TAG                         "BackingUp"

#define FLOAT_IS_ZERO(x)                       (abs(x)<1.0e-5)
//#define BACKING_UP_CALIBR
#define CALIBRA_DISTANCE                    0.218

#define RAD_TO_DEGREE                          (180/M_PI)
//obj detect
#define DETECT_RETRY_TIMES               3
#define DETECT_TIMEOUT                        (3000)//unit ms
#define DETECT_ROLL_TIME                   (6)
#define DETECT_ROLL_ANGLE               (2*M_PI/DETECT_ROLL_TIME)
#define DETECT_ROLL_SPEED               (1.5)
#define DETECT_ROLL_TIMEOUT             (3000) // ms
#define DETECT_MOVE_SPEED              (0.15)
#define DETECT_STOP_DISTANCE        (0.3)
#define DETECT_STOP_ERROR               (0.1)
#define DETECT_NOMALISE_DISTANCE    (0.3f)
#define DETECT_NORMALISE_HEIGHT   (0.264)  //the height/PIC_HEIGHT of charging pile while the disance between charge pile and  car is DETECT_NOMALISE_DISTANCE,a coarse value
#define CAR_LENGTH                                   (0.1)

#define GOTO_IFCANCELD if(canceled() || mCharging) { goto out;}
#define RETURN_IFCANCELD if(canceled() || mCharging) { return 0;}

//BACKING
#define BACK_ROLL_ERROR                      (1.5*M_PI/180) //1 degree
#define BACK_ROLL_SPEED                          1.0f
#define BACK_ROLL_ANGLE                       (175*M_PI/180)
#define BACK_ROLL_TIMEOUT                  (20000)

#define ALIGN_MAX_TIMES                        60
#define ALIGN_MAX_DETECT_FAIL_TIMES       100
#define ALIGN_ROLL_SPEED                          1.5f
#define ALIGN_MOVE_SPEED                          0.1f
#define ALIGN_DISTANCE                          0.16  //0.20
#define ALIGN_X_ERROR                           0.01
#define ALIGN_Z_ERROR                           0.05
#define ALIGN_GOOD_ESTIMATE_ROLL        (10*M_PI/180)
#define ALIGN_GOOD_ESTIMATE_ANGLE     (15*M_PI/180)

#define NAV_OBSTACLE_MIN_DIST                   0.2
#define NAV_OBSTACLE_CNT                               5
#define NAV_OBSTACLE_MAX_DIST                   0.5
#define NAV_OBSTACLE_FORWARD_DIST       0.4
#define NAV_OBSTACLE_ANGLE              (10.0*M_PI/180)


#define CHESSBAORD_ROW                      5
#define CHESSBAORD_COL                        5
#define CHESSBAORD_SIZE                       0.0065

#define DETECT_COUNT                             2
#define MAX_ALIGN_COUNT                 100

#ifdef APP_ARCH_X86
#define CAM_K_FX                                        802.8910929097393
#define CAM_K_FY                                        799.1434042680177
#define CAM_K_CX                                        311.14325635108935
#define CAM_K_CY                                        278.42741847426544
#define DISTRO_K1                                       0.029738668846275632
#define DISTRO_K2                                       0.14594602101053278
#define DISTRO_P1                                       0.010307430942468115
#define DISTRO_P2                                       -0.005671263595704292
#else

#define CAM_K_FX                                        720.5631606
#define CAM_K_FY                                        723.6323653
#define CAM_K_CX                                        659.39411452
#define CAM_K_CY                                        369.00731648
#define DISTRO_K1                                       -0.36906219
#define DISTRO_K2                                       0.16914887
#define DISTRO_P1                                       -0.00147033
#define DISTRO_P2                                       -0.00345135

#endif

#define MAX_BACK_UP   5

#define MIN_X_DIST (0.8e-2)
#define MIN_Z_DIST (4.0e-2)
#define MIN_ANGLE  (3.6e-2)
#define MOVE_DIST  0.4

#define CHARGING_DETECT_TIMES   (30*60)
#define VOL_AVG_TIMES   10

#define X_IS_ZERO(x)                       (abs(x)<MIN_X_DIST)
#define Z_IS_ZERO(x)                       (abs(x)<MIN_Z_DIST)
#define ANGLE_IS_ZERO(x)            (abs(x)<MIN_ANGLE)

#define RETURN_IFCANCELED if (canceled()) { return; }
namespace roller_eye{
    static const string HOME="home";
    enum BackingStatus{
        BACKING_STATUS_OK=0,
        BACKING_ERROR_MOVE,
        BACKING_DETECT_NO_HOME,
        BACKING_DETECT_APPROACH,
        BACKING_DETECT_CANCEL
    };

    BackingUp::BackingUp(ros::NodeHandle& handle):
    mPub(handle,true,"backing_up_status"),
    mBackupRoll(0.0),
    mCVStream(ALIGN_IMG_WIDTH,ALIGN_IMG_HEIGHT),
    mROIValid(false),
    mIsTracing(false),
    mAdjusting(false),
    mCanceled(false),
    mReconnecting(false),
    mLastVoltage(0),
    mChargingCounter(0),
    mCurrentVoltage(0)
    {
        mAngle = 0.0;
        mBatteryStatus = mGlobal.subscribe("/SensorNode/simple_battery_status",10,&BackingUp::onBatteryStatus,this);
        mStatusPub=  handle.advertise<std_msgs::Int32>("going_home_status",1);   //订阅该主题不会启动回家功能
        mCancel=handle.advertiseService("nav_cancel",&BackingUp::cancel,this);
        mCmdVel=mGlobal.advertise<geometry_msgs::Twist>("cmd_vel",100);
        mAdjustClient = mGlobal.serviceClient<adjust_ligth>("/CoreNode/adjust_light");
        mNightGetClient = mGlobal.serviceClient<night_get>("CoreNode/night_get");

        PLOG_INFO(BACKING_UP_TAG,"Init backing up\n");
        PLOG_INFO(BACKING_UP_TAG,"updateStatus(status::BACK_UP_INACTIVE)\n");
        updateStatus(status::BACK_UP_INACTIVE);

        mTimer=mGlobal.createTimer(ros::Duration(1),&BackingUp::timerCallback,this);
        mTestSub = mGlobal.subscribe("/CoreNode/alg_test",1,&BackingUp::test,this);

        mTestMoveByObj = mGlobal.subscribe("/CoreNode/testMoveByObj", 1, &BackingUp::testMoveByObj, this);
        mTestMoveRoll = mGlobal.subscribe("/CoreNode/testMoveRoll", 1, &BackingUp::testMoveRoll, this);
        mTestBackup = mGlobal.subscribe("/CoreNode/testBackup", 1, &BackingUp::onTestBackup, this);
    }

    void BackingUp::testMoveByObj(const geometry_msgs::Pose::ConstPtr& msg){
        tAngle = msg->position.x;
        tz = msg->position.y;

        tcx = msg->orientation.x;
        tcz = msg->orientation.y;
        tcangle = msg->orientation.z;


        PLOG_INFO("TEST_MoveByObj", "angle: %f, z: %f", tAngle, tz);
        auto th = std::thread([this](){
            if (tcx == 0 && tcz==0 && tcangle==0){
                if(tz){
                    mAlgo.move(0, tz, ALIGN_MOVE_SPEED);
                }
                if(tAngle) moveByObj2(tAngle, 0);
            }
            else if (tAngle == 0.0 && tz == 0.0){
                auto m = moveByCorner(tcx,tcz,tcangle);
            }
        });
        th.detach();
    }

    void BackingUp::testMoveRoll(const geometry_msgs::Pose::ConstPtr& msg){
        mPoseMsg = *msg;

        PLOG_INFO(BACKING_UP_TAG, "test move and roll");
        auto th = std::thread([this](){
            if(mPoseMsg.position.x){
                float mspeed = mPoseMsg.position.y ? mPoseMsg.position.y : ALIGN_MOVE_SPEED;
                mAlgo.move(0, mPoseMsg.position.x, mspeed);
            }
            else if (mPoseMsg.orientation.z)
            {
                float rspeed = mPoseMsg.orientation.y ? mPoseMsg.orientation.y : ALIGN_ROLL_SPEED;
                auto pdictLeft = make_shared<DistanceMap>();
                mAlgo.getDistance(rspeed, pdictLeft);
            }
            else if (mPoseMsg.orientation.w)
            {
                float rspeed = mPoseMsg.orientation.y ? mPoseMsg.orientation.y : ALIGN_ROLL_SPEED;
                float minW = mPoseMsg.orientation.x ? mPoseMsg.orientation.x : ALIGN_ROLL_SPEED;
                mAlgo.rollEx(mPoseMsg.orientation.w, rspeed, minW, BACK_ROLL_TIMEOUT, BACK_ROLL_ERROR);
            }

        });
        th.detach();
    }


    /* msg.data == 0: getHomeDistanceAndAngle then return
    *  msg.data == 1: getHomeDistanceAndAngle -> Check backup, doBackup or moveByCorner once
    *  msg.data == 3: only roll 180 degrees
    */
    void BackingUp::onTestBackup(const std_msgs::Int8::ConstPtr& msg)
    {
        auto msg_ = *msg;
        PLOG_INFO(BACKING_UP_TAG, "test backup");
        auto th = std::thread([this, msg_](){
    		DeviceDefaultConfig cfg;
            if (msg_.data == 4) {
                PLOG_INFO(BACKING_UP_TAG, "test only doDetect()");
                doDetect();
                return;
            }
            if(msg_.data == 2) {
                PLOG_INFO(BACKING_UP_TAG, "test only doBackup()");
                doBackup();
                return;
            }
            if(msg_.data == 3) {
                PLOG_INFO(BACKING_UP_TAG, "test only 180 degrees rotation");
                float mBackupRoll = cfg.getTrackTypeBackupRollAngle() * M_PI/180.0;
                float rollSpeed = cfg.getTrackTypeBackupRollSpeed();
                float minW = cfg.getTrackTypeBackupRollMinSpeed();
                mAlgo.rollEx(mBackupRoll, rollSpeed, minW,  /// 1.0,
                       BACK_ROLL_TIMEOUT,BACK_ROLL_ERROR);
                usleep(100*1000);
                return;
            }
            AlgoOBjPos objPos;
            float lastX = 0;
            float lastZ = 0;
            float angle = 0;
            float ratio = 0;
            if(waitObj(HOME, DETECT_TIMEOUT, objPos, 0) != 0){
                PLOG_ERROR(BACKING_UP_TAG, "Cannot find the object. Quit!");
                return;
            }
            if(!getHomeDistanceAndAngle(objPos, lastX, lastZ, angle)) {
                PLOG_ERROR(BACKING_UP_TAG, "Cannot getHomeDistance and Angle. Quit!");
                return;
            }
            PLOG_INFO(BACKING_UP_TAG, "[onTestBackup]x: %f, z: %f, angle: %f", lastX, lastZ, angle*180.0/M_PI);
            if(msg_.data == 0){
                return;
            }
            if(canBackup(lastX, lastZ, angle)){
                PLOG_INFO(BACKING_UP_TAG, "DO BACKUP");
                doBackup();
            }
            else {
                PLOG_DEBUG(BACKING_UP_TAG, "[TestBackUp] moveByCorner: x: %f, z: %f, angle: %f!", lastX, lastZ, angle);
                angle = angle * ratio;
                PLOG_DEBUG(BACKING_UP_TAG, "[TestBackUp] use ratio: %f, angle-->: %f!", ratio, angle);
                moveByCorner(lastX, lastZ, angle);
            }
        });
        th.detach();
    }

    BackingUp::~BackingUp()
    {

    }
    void BackingUp::start()
    {
        mCanceled = false;
        mCharging = false;
        PLOG_INFO(BACKING_UP_TAG,"start backing up\n");
        mThread=std::thread(&BackingUp::backupLoop,this);
    }
    void BackingUp::stop()
    {
        PLOG_INFO(BACKING_UP_TAG,"stop backing up\n");
        mAlgo.quitAlgo();
        deinitEstimate();
        updateStatus(status::BACK_UP_INACTIVE);
        if(mThread.joinable()){
            mThread.join();
        }
        mCanceled = false;
    }
    int BackingUp::setData(std_msgs::Int32& s)
    {
        s.data=mStatus;
        return 0;
    }

    bool BackingUp::adjustLight(int cmd)
    {
        adjust_ligth req;
        req.request.cmd = cmd;
        if(!mAdjustClient.call(req)){
            return false;
        }

        return true;
    }

    bool BackingUp::isNightMode()
    {
        night_get::Request req;
        night_get::Response res;
        if (mNightGetClient.call(req,res)){
            return res.isNight;
        }

        return false;
    }
    int BackingUp::waitObj(const string& name,int timeout,AlgoOBjPos &pos, int cmd)
    {
        if (isNightMode()){
            timeout *= 1.5;
        }

        int ret = mAlgo.waitObj(HOME, timeout, pos);
        PLOG_DEBUG(BACKING_UP_TAG, "ret: %d", ret);
        if(0 == ret){
            PLOG_DEBUG(BACKING_UP_TAG, "objPose: left: %d, top: %d, right: %d, bottom: %d, width: %d, height: %d", pos.left, pos.top, pos.right, pos.bottom, pos.width, pos.height);
            return ret;
        }
        return ret;
    }
    void BackingUp::test(const std_msgs::Int32::ConstPtr& msg)
    {
        PLOG_DEBUG(BACKING_UP_TAG,"test msg->data: %d ",msg->data);
        switch (msg->data)
        {
            case 0: {
                if (!mReconnecting){
                    auto th=std::thread([this](){
                            mReconnecting = true;
                            reConnectToChargePile();
                            mReconnecting = false;
                        });
                    th.detach();
                }
            }
                break;
            default:
                break;
        }
    }
    void BackingUp::timerCallback(const ros::TimerEvent& evt)
    {
        std::ifstream chargeFile("/sys/class/power_supply/rk-bat/status");
        std::ifstream voltageFile("/sys/class/power_supply/rk-bat/voltage_now");
        int status[3];
        int voltage;
        string charge;
        chargeFile>>charge;
        voltageFile>>voltage;
        if(chargeFile.fail()||voltageFile.fail()){
            return;
        }

        static int avgVoltage = 0;
        if(charge != "Charging"){
            avgVoltage                  = 0;
            mLastVoltage             = 0;
            mCurrentVoltage      = 0;
            mChargingCounter  = 0;
           return;
        }

        if (mReconnecting){
            return;
        }

        mChargingCounter++;
        int detectStart = CHARGING_DETECT_TIMES-VOL_AVG_TIMES;
        int detectEnd  = CHARGING_DETECT_TIMES;
        if (mChargingCounter<=VOL_AVG_TIMES){
            avgVoltage += voltage/VOL_AVG_TIMES;
        }else if (mChargingCounter>detectStart &&
                        mChargingCounter<=detectEnd){
            mCurrentVoltage += voltage/VOL_AVG_TIMES;
        }else if (mChargingCounter > detectEnd){
            if (0 == mLastVoltage){
                mLastVoltage = avgVoltage;
            }
            if (mCurrentVoltage < 0.85*mLastVoltage){
                  auto th=std::thread([this](){
                    mReconnecting = true;
                    reConnectToChargePile();
                    mReconnecting = false;
                });
                th.detach();
            }else if (mCurrentVoltage > mLastVoltage){
                mLastVoltage = mCurrentVoltage;
            }
            avgVoltage                   = 0;
            mCurrentVoltage      = 0;
            mChargingCounter  = 0;
        }
    }

    void BackingUp::reConnectToChargePile()
    {
        mAlgo.reConnectToChargePile(1000);
    }

    void BackingUp::backupLoop()
    {
        PLOG_ERROR(BACKING_UP_TAG,"AlgoUtils %s %d\n",__FILE__, __LINE__);
        mCanceled = false;
        PLOG_INFO(BACKING_UP_TAG,"updateStatus(status::BACK_UP_DETECT)\n");
        updateStatus(status::BACK_UP_DETECT);
        mAlgo.resetQuitFlag();
        mAlignCounter      = 0;
        mBackupCounter = 0;
        bool run = true;
        while(run && !canceled()){
            switch (mStatus)
            {
            case status::BACK_UP_DETECT:
                PLOG_ERROR(BACKING_UP_TAG,"BackingUp::doDetect \n");
                adjustLight(1);
                doDetect();
                PLOG_ERROR(BACKING_UP_TAG,"BackingUp::doDetect end\n");
                break;
            case status::BACK_UP_ALIGN:
                PLOG_ERROR(BACKING_UP_TAG,"BackingUp::doAlign*************** \n");
                doAlign();
                // run = false;
                PLOG_ERROR(BACKING_UP_TAG,"BackingUp::doAlign end***********\n");
                break;
            case status::BACK_UP_BACK:
                PLOG_ERROR(BACKING_UP_TAG,"BackingUp::doBackup \n");
                adjustLight(1);
                doBackup();
                PLOG_ERROR(BACKING_UP_TAG,"BackingUp::doBackup end\n");
                break;
            case status::BACK_UP_SUCCESS:
            case status::BACK_UP_FAIL:
            case status::BACK_UP_INACTIVE:
            case status::BACK_UP_CANCEL:
                PLOG_INFO(BACKING_UP_TAG,"backupLoop quit\n");
                run=false;
                break;
            default:
                break;
            }
        }
        adjustLight(2);
        usleep(50*1000);
        mCVStream.stopStream();
        mCanceled = false;
        PLOG_INFO(BACKING_UP_TAG,"backing up exit,status=%d\n",mStatus);
    }

    bool BackingUp::getCameraPose(float z,Eigen::Vector3d& pos,cv::Rect2f& roi, float &xdist, float &zdist,float &angle)
    {
        cv::Mat grey;
         if(mCVStream.getCVImg(grey)<0){
            return false;
        }
        cv::Rect rect(round(roi.x*grey.cols),round(roi.y*grey.rows),round(roi.width*grey.cols),round(roi.height*grey.rows));

        rect.x=std::max(0,rect.x);
        rect.x=std::min(rect.x,grey.cols-1);
        rect.width=std::min(rect.width,grey.cols-rect.x);
        rect.y=std::max(0,rect.y);
        rect.y=std::min(rect.y,grey.rows-1);
        rect.height=std::min(rect.height,grey.rows-rect.y);

        return calcCameraPoseEx(z,grey,pos,rect, xdist, zdist, angle);
    }

    bool BackingUp::getHomeDistanceAndAngle(const AlgoOBjPos &objPos,float &x,float&z,float &angle)
    {
        int i;
        bool bRet = false;
        Eigen::Vector3d position;
        //pos = 1.5pos
        float scale = 0.8;
        AlgoOBjPos pos = objPos;
        pos.left = static_cast<int> (scale*pos.left);
        pos.top = static_cast<int> (scale*pos.top);
        pos.bottom = static_cast<int> (pos.bottom/scale);
        pos.right = static_cast<int> (pos.right/scale);

        pos.bottom = pos.bottom>pos.height ? pos.height : pos.bottom;
        pos.right = pos.right>pos.width ? pos.width : pos.right;
        cv::Rect2f rio((float)pos.left/pos.width,(float)pos.top/pos.height,(float)(pos.right-pos.left)/pos.width,(float)(pos.bottom-pos.top)/pos.height);

        for(i=0;i<2;i++){
            bRet = getCameraPose(0,position,rio, x, z, angle);
            if(bRet){
                    break;
            }
            usleep(100*1000);
        }

        if(bRet){
            mAngle = angle;
            mROI=rio;
            mROIValid=true;
        }
        PLOG_DEBUG(BACKING_UP_TAG,"getHomeDistanceAndAngle detect mangle=%f, x=%f, z=%f, angle=%f (degrees)\n",mAngle,x,z,angle*180.0/M_PI);
        return bRet;
    }

    bool BackingUp::getHomeDistanceAndAngleByObj(AlgoOBjPos &pos,float &angle, float&z)
    {
        int centerX = (pos.left+pos.right)/2;
        int height   = pos.bottom-pos.top;
        int width    = pos.right-pos.left;

//1.2m H-36
//0.6m H-89
#define  A_COEFF   1.047197
#define TAN_60      1.7321
#define  F_COEFF (0.6*89/36)
#define X_CENTER 640
#define X_CENTER_LEFT 600
#define X_CENTER_RIGHT 680
#define NEAR_HEIGHT   160
#define NEAREST_HEIGHT   220
#define NEAR_Z   0.3

        do{
             if (height>NEAREST_HEIGHT){
                 angle = 0;
                z = -0.1;
                break;
            }

            z = 36*F_COEFF / height;
            z -= ALIGN_DISTANCE/100;
            if (z<NEAR_Z){
                z=0.0;
            }

            if (height>NEAR_HEIGHT &&
                    centerX>X_CENTER_LEFT &&
                    centerX<X_CENTER_RIGHT){
                PLOG_DEBUG(BACKING_UP_TAG,
                            "BackingUp::getHomeDistanceAndAngleByObj height:%d, centerX:%d\n",
                            height,  centerX);
                return false;
            }

            if (centerX>=X_CENTER_LEFT &&
                   centerX<=X_CENTER_RIGHT){
                angle = 0;
                break;
            }
            if (centerX<X_CENTER_LEFT){
                angle = atan( A_COEFF * (1.0-float(centerX)/ X_CENTER));
                PLOG_DEBUG(BACKING_UP_TAG,
                        "0 getHomeDistanceAndAngleByObj centerX:%d , %f\n", centerX,1.0-float(centerX)/ X_CENTER );
            }else{
                angle = -atan(A_COEFF * (float(centerX-X_CENTER))/ X_CENTER);
                PLOG_DEBUG(BACKING_UP_TAG,
                        "1 getHomeDistanceAndAngleByObj centerX:%d , %f\n", centerX,float(centerX-X_CENTER)/ X_CENTER );
            }
        }while(0);

        PLOG_DEBUG(BACKING_UP_TAG,
                "BackingUp::getHomeDistanceAndAngleByObj 1 angle:%f, z:%f\n",  angle,  z);
        return true;
    }

    int BackingUp::detectOnce()
    {
        int i;
        int timeout = 1.5*DETECT_TIMEOUT;
		int detectTimes =  DETECT_ROLL_TIME;
		double  detectRollAngle = 2.0*M_PI/double(detectTimes);
		double  detectRollSpeed = DETECT_ROLL_SPEED;
 		if (PltConfig::getInstance()->isTrackModel())	/// Track model ...
		{
			detectTimes = 9;
			detectRollAngle = 2.0*M_PI/double(detectTimes);
			detectRollSpeed = 1.5 ;
		}
        for(i=0;i<detectTimes && !canceled() && !mCharging;i++){
            if(waitObj(HOME, timeout, mObjPos,1)==0 ){
                break;
            }
			if (!PltConfig::getInstance()->isTrackModel())	/// Track model,don't change detect time ...
	        {
                timeout = DETECT_TIMEOUT;
            }
            int roll = mAlgo.rollAndFindPile(detectRollAngle,detectRollSpeed, DETECT_ROLL_TIMEOUT);
            if(roll == -1){
                return BACKING_ERROR_MOVE;
            }
            else if (roll == 1) {
                PLOG_DEBUG(BACKING_UP_TAG, "Cancel detect!");
                return BACKING_DETECT_CANCEL;
            }
        }

        if(canceled() || mCharging){
            PLOG_DEBUG(BACKING_UP_TAG, "Cancel detect!");
            return BACKING_DETECT_CANCEL;
        }
        if(i==detectTimes){
			if(waitObj(HOME, timeout, mObjPos,1)!=0 )   ///Do last try...
            	return BACKING_DETECT_NO_HOME;
        }

        PLOG_ERROR(BACKING_UP_TAG,"BackingUp::detectOnce() BACKING_STATUS_OK \n");
        return BACKING_STATUS_OK;
    }

    void BackingUp::doDetect()
    {
        int ret;
        mAlgo.waitObjBegin();
        for (int i=0; i<DETECT_COUNT; i++){
            PLOG_DEBUG(BACKING_UP_TAG,"doDetect i=%d",i);
            ret = detect(i+1);
            if (BACKING_DETECT_CANCEL == ret)  {
                goto out;
            }else if (BACKING_STATUS_OK == ret){
                updateStatus(status::BACK_UP_ALIGN);
                goto out;
            }
        }
        updateStatus(status::BACK_UP_FAIL);

    out:
        mAlgo.waitObjEnd();
    }

    int BackingUp::detect(int times)
    {
        int ret = BACKING_DETECT_NO_HOME;
        int size = MOVE_DIST/0.02+1;
        float y = 0.0;
        TrackList tlst;
        for (int j=0; j<size; j++){
            y = (size-j)*0.02;
            Eigen::Vector3d pos=Eigen::Vector3d(0.0,y,0.0);
            Eigen::Quaterniond pose =Eigen::Quaterniond(1.0,0.0,0.0,0.0);
            TrackPoint point(ros::Time::now().toNSec(),pos,pose,0,0.2,0);
            tlst.push(point);
        }

        for (int j=0; j<6; j++){
            if (mCharging) {
                PLOG_DEBUG(BACKING_UP_TAG, "Cancel detect!");
                ret = BACKING_DETECT_CANCEL;
                goto out;
            }
            for (int k=0; k<times; k++){
                PLOG_DEBUG(BACKING_UP_TAG,"detect j=%d, k=%d, times=%d",j,k,times);
                ret=detectOnce();
                if(ret==BACKING_STATUS_OK || ret == BACKING_DETECT_CANCEL){
                    goto out;
                }else if(ret==BACKING_DETECT_NO_HOME){
                    auto track=make_shared<TrackList>();
                    *track=tlst;
                    mTrace.setTrackList(track);
                    if (startTrace(true)){
                        mIsTracing = true;
                        PLOG_INFO(BACKING_UP_TAG,"BackingUp::doDetect mIsTracing %d\n",mIsTracing);
                        while (mIsTracing){
                            if (canceled() || mCharging){
                                stopTrace();
                                ret = BACKING_DETECT_CANCEL;
                                goto out;
                            }
                            if(mDetectObstacleCnt>NAV_OBSTACLE_CNT){
                                PLOG_DEBUG(BACKING_UP_TAG,"doDetect Detect obstacle");
                                mAdjusting=true;
                                AlgoOBjPos pos;
                                if(waitObj(HOME, DETECT_TIMEOUT, pos,0)==0){
                                    mAdjusting=false;
                                    stopTrace();
                                    ret = BACKING_STATUS_OK;
                                    goto out;
                                }
                                if (canceled() || mCharging){
                                    stopTrace();
                                    ret = BACKING_DETECT_CANCEL;
                                    goto out;
                                }
                                avoidObstacle();
                                stopTrace();
                                mAdjusting=false;
                                break;
                            }
                            usleep(20*1000);
                        }
                    }
                }
                else if (ret == BACKING_ERROR_MOVE) {
                    PLOG_DEBUG(BACKING_UP_TAG, "Roll for detect home fail, move out a little bit");
                    move(0.0, 0.15, DETECT_MOVE_SPEED);
                }
            }
            int roll = mAlgo.rollAndFindPile(M_PI/3,DETECT_ROLL_SPEED);
            if (roll == 1) {
                PLOG_DEBUG(BACKING_UP_TAG, "Cancel detect!");
                ret = BACKING_DETECT_CANCEL;
                goto out;
            }
        }
        ret = BACKING_DETECT_NO_HOME;

    out:
       return ret;
    }

     void BackingUp::doAlign()
    {
       PLOG_DEBUG(BACKING_UP_TAG,"BackingUp::doAlign() waitObj\n");

        mAlgo.waitObjBegin();

        float lastX = 0;
        float lastZ = 0;
        float angle = 0;

        bool bFound = false;
        bool bFirst = true;
        bool bBackup = false;
        bool bChess = false;
        bool bAdjust = false;
        bool bMoveLeft = false;
        int    nMoveT = 0;
        int    nMoveCnt = 0;
        do {
            if (nMoveCnt++ > MAX_ALIGN_COUNT){
                PLOG_DEBUG(BACKING_UP_TAG,"break because nMoveCnt(%d) > %d \n", nMoveCnt, MAX_ALIGN_COUNT);
                break;
            }
            if(bFirst ||
              (waitObj(HOME,DETECT_TIMEOUT,mObjPos, 0)==0)){
                bFound = true;
                bMoveLeft = bFirst = false;

                if (nMoveT>0){
                    if (!bAdjust){
                        adjustLight(0);
                        bAdjust = true;
                    }
                    bChess = getHomeDistanceAndAngle(mObjPos,lastX,lastZ,angle);
                    PLOG_INFO(BACKING_UP_TAG,
                        "getHomeDistanceAndAngle(x:%f, z:%f, angle:%f)= %d \n",  lastX,  lastZ, angle,bChess);
                    // break;
                    if (bChess){
                        PLOG_DEBUG(BACKING_UP_TAG,
                            "BackingUp::doAlign() x:%f, z:%f, angle:%f\n",  lastX,  lastZ, angle);
                        if (canBackup(lastX, lastZ, angle)){
                            bBackup = true;
                            break;
                        }

                        nMoveT = moveByCorner(lastX, lastZ, angle);
                        GOTO_IFCANCELD
                        continue;
                    }else{
                        adjustLight(0);
                    }
                }
                GOTO_IFCANCELD

                if (getHomeDistanceAndAngleByObj(mObjPos, angle,lastZ)){
                    nMoveT = moveByObj(angle,lastZ);
                    // nMoveT = 2;
                    continue;
                }

                if (nMoveT>0){
                    break;
                }
                if (!bMoveLeft){
                    moveByCorner(0.15,-0.05,0.0);
                    bMoveLeft = true;
                    nMoveT = 0;
                    continue;
                }

            }else if (!bFound){
                if (!bMoveLeft) {
                    break;
                }
                moveByCorner(-0.3, 0.0, 0.0);
                bMoveLeft = false;
            }

            if (2 == nMoveT){
                moveByObj(-angle/2.0,-lastZ/2.0);
            }else if (1==nMoveT){
                moveByCorner(-lastX/2.0, -lastZ/2.0, -angle/2.0);
			if (PltConfig::getInstance()->isTrackModel())	/// Track model ...
				if (canBackup(lastX, lastZ, angle)){
					PLOG_ERROR(BACKING_UP_TAG, "Can Backup 002!!!");
					bBackup = true;
					break;
				}
            }else if (bMoveLeft){
                moveByCorner(-0.3, 0.0, 0.0);
                bMoveLeft = false;
            }
            GOTO_IFCANCELD
            bFound = false;
            PLOG_DEBUG(BACKING_UP_TAG,"found home once \n",lastX,lastZ);

        }while(true);

        if (!bBackup){
            if (mAlignCounter++ < 3){
                updateStatus(status::BACK_UP_DETECT);
            }else{
                updateStatus(status::BACK_UP_FAIL);
            }
            goto out;
        }
        mAlignCounter = 0;
        updateStatus(status::BACK_UP_BACK);
out:
        mAlgo.waitObjEnd();
        PLOG_ERROR(BACKING_UP_TAG,"BackingUp::doAlign() quit\n");
    }

    void BackingUp::doBackup()
    {
        RETURN_IFCANCELED
		double moveDist = 0.04, rollSpeed= 3.50*BACK_ROLL_SPEED, minW = 1.0;
        double angle_ratio = 1.0;
		// DeviceDefaultConfig cfg;
        if (PltConfig::getInstance()->isTrackModel())  /// Track model ...
		{
			minW = mCfg.getTrackTypeBackupRollMinSpeed();
		    rollSpeed = mCfg.getTrackTypeBackupRollSpeed();
		    mBackupRoll = mCfg.getTrackTypeBackupRollAngle() * M_PI/180.0;
        } else {
            rollSpeed = mCfg.getBackupRollSpeed();
            mBackupRoll = mAngle + BACK_ROLL_ANGLE;
        }
        angle_ratio = mCfg.getAngleRatio();
		moveDist = mCfg.getBackupMoveDist();

        mAlgo.move(0, moveDist,ALIGN_MOVE_SPEED);

        PLOG_DEBUG(BACKING_UP_TAG,"mAngle: %f (degrees), backup roll angle:%f (degrees)\n", mAngle*angle_ratio*180.0/M_PI, mBackupRoll*180.0/M_PI);
        PLOG_DEBUG(BACKING_UP_TAG,"mBackupRoll: %f, rollSpeed: %f, minW: %f\n", mBackupRoll, rollSpeed, minW);
        if(mAlgo.rollEx(mBackupRoll, rollSpeed, minW,  /// 1.0,
                       BACK_ROLL_TIMEOUT,BACK_ROLL_ERROR)<0){
            updateStatus(status::BACK_UP_FAIL);
            return;
        }
        RETURN_IFCANCELED
        sleep(1);
        RETURN_IFCANCELED
        mAlgo.action(0.0,-0.1,0.0,1800);

        RETURN_IFCANCELED

		PLOG_INFO(BACKING_UP_TAG,"( Now connecting to chargePile...)\n");

        int tryCnt;
        int connectToChargePile_ = -1;
        if (PltConfig::getInstance()->isTrackModel()) {  /// Track model ...
            tryCnt = mCfg.getTrackTypeTryCount();
            PLOG_INFO(BACKING_UP_TAG, "Track model, tryCnt: %d", tryCnt);
            connectToChargePile_ = mAlgo.connectToChargePileTrack(tryCnt);
        } else { /// Mecanum model...
            tryCnt = mCfg.getTryCount();
            PLOG_INFO(BACKING_UP_TAG, "Mecanum model, tryCnt: %d", tryCnt);
            connectToChargePile_ = mAlgo.connectToChargePile(tryCnt, mBackupCounter);
        }

        if(connectToChargePile_ == 0){
			PLOG_INFO(BACKING_UP_TAG,"updateStatus(status::BACK_UP_SUCCESS)\n");
            updateStatus(status::BACK_UP_SUCCESS);
        }else{
            RETURN_IFCANCELED
            if (++mBackupCounter < MAX_BACK_UP){
                mAlgo.move(0, 0.25,ALIGN_MOVE_SPEED);
                RETURN_IFCANCELED
                roll(M_PI,ALIGN_ROLL_SPEED);
                //updateStatus(status::BACK_UP_DETECT);
                updateStatus(status::BACK_UP_ALIGN);
               //pubStatus(status::BACK_UP_REDETECT);  //for test backup rate of success;
            }else{
                updateStatus(status::BACK_UP_FAIL);
            }
        }
    }

    bool BackingUp::canBackup(float x, float z, float angle)
    {
        float xOffset = mCfg.getAlignXOffset();
        float minX = mCfg.getAlignMinX();
        float minZ = mCfg.getAlignMinZ();
        float minAngle = mCfg.getAlignMinAngle();
        if (abs(z) <= abs(minZ) &&
            abs(x - xOffset) <= abs(minX) &&
            abs(angle) <= abs(minAngle)){
			PLOG_WARN(BACKING_UP_TAG,"canBackup(),Got it!!!  angle:%f, x: %f, z: %f\n",angle,x,z);
            return true;
        }
        return false;
    }

    int BackingUp::moveByCorner2(float x,  float z, float angle)
    {
	   PLOG_INFO(BACKING_UP_TAG,"moveByCorner2() angle:%f, x: %f, z: %f\n",angle*180.0/M_PI,x,z);

        double sign = 1.0;
        sign = (x>0) ? 1.0 : -1.0;
        if(z>0.15 || abs(angle)> M_PI*4.0/9.0){
            PLOG_WARN(BACKING_UP_TAG, "The angle too large, back to the center...");
            roll(sign*M_PI/3.0, ALIGN_ROLL_SPEED, DURATION_10_S, DGREE_3_RAD);
            usleep(100*1000);
            move(0.0, sign*x, ALIGN_MOVE_SPEED, DURATION_10_S, 0.005);
            usleep(100*1000);
            roll(-sign*(M_PI*140/180), ALIGN_ROLL_SPEED, DURATION_10_S, DGREE_3_RAD);
            usleep(100*1000);
            return 1;
        }

        float t_angle = sign*M_PI/2.0 + angle;
        // DeviceDefaultConfig cfg;
        float roll_factor = mCfg.getTrackTypeFloorRollFactor();
        float align_x_offset = mCfg.getAlignXOffset();
        mAlgo.rollEx(t_angle, ALIGN_ROLL_SPEED + roll_factor, 2.0, DURATION_10_S, DGREE_2_RAD);
        usleep(100*1000);
        RETURN_IFCANCELD
        if(!X_IS_ZERO(x)){
            if (abs(x)<2*MIN_X_DIST){
                if (x<0){
                    x -= MIN_X_DIST/2;
                }else{
                    x += MIN_X_DIST/2;
                }
            }
            if(abs(x)< CAR_LENGTH)
                    move(0.0,sign*(x-align_x_offset),ALIGN_MOVE_SPEED,DURATION_10_S,0.005);
            else
                    move(0.0,sign*(x-align_x_offset),ALIGN_MOVE_SPEED);
            usleep(100*1000);
            RETURN_IFCANCELD
        }
        mAlgo.rollEx(-sign*M_PI/2.0,ALIGN_ROLL_SPEED + roll_factor, 2.0, DURATION_10_S,DGREE_2_RAD);
        usleep(100*1000);

	   if (!Z_IS_ZERO(z)){
		   if(abs(z)< CAR_LENGTH)
		   		move(0.0,-z,ALIGN_MOVE_SPEED,DURATION_10_S,0.005);
		   else
		   		move(0.0,-z,ALIGN_MOVE_SPEED);
		   usleep(100*1000);
	   }

	   return 1;
   }


   int BackingUp::moveByCorner3(float x,  float z, float angle)
    {
        PLOG_INFO(BACKING_UP_TAG,"moveByCorner3() angle:%f, x: %f, z: %f\n",angle,x,z);
		double s = 0.0;
		double sign = 1.0;
        // DeviceDefaultConfig cfg;
        float roll_factor = mCfg.getTrackTypeFloorRollFactor();
		if (x < 0)
		  sign = -1.0 ;

		if((abs(angle)> 2.0*MIN_ANGLE)
			|| (abs(x)> MIN_X_DIST*2.0)
			)
		{
            angle -=  sign*M_PI/6.0;
            if (!ANGLE_IS_ZERO(angle)){
                RETURN_IFCANCELD
                if (abs(angle)<2*MIN_ANGLE){
                    if (angle<0){
                        angle -= MIN_ANGLE/2.0;
                    }else{
                        angle += MIN_ANGLE/2.0;
                    }
                }
                mAlgo.rollEx(angle,ALIGN_ROLL_SPEED + roll_factor, 2.0, DURATION_10_S,DGREE_2_RAD);
                usleep(100*1000);
                RETURN_IFCANCELD
            }
		}
		else
		{   /// just back and retry...
			if( z< -MIN_Z_DIST
				){

				    move(0.0,-z-MIN_X_DIST,ALIGN_MOVE_SPEED,DURATION_10_S,0.005);
			        usleep(100*1000);
			        return 1;
                }
		}
        if (!X_IS_ZERO(x)){
				{
            	 s = x / sin(sign*M_PI/6.0);

				if (abs(s)<2*MIN_X_DIST){
					if (s<0){
						s -= MIN_X_DIST/2;
					}else{
						s += MIN_X_DIST/2;
					}
				}
				if(abs(s)< CAR_LENGTH)
					 move(0.0,-s,ALIGN_MOVE_SPEED,DURATION_10_S,0.005);
				else
					 move(0.0,-s,ALIGN_MOVE_SPEED);
            	usleep(100*1000);
            	RETURN_IFCANCELD
			}
        }
		angle = sign*M_PI/6.0;
        RETURN_IFCANCELD
        mAlgo.rollEx(angle,ALIGN_ROLL_SPEED + roll_factor, 2.0, DURATION_10_S,DGREE_2_RAD);
        usleep(100*1000);
        RETURN_IFCANCELD
        z -= s*cos(angle);
        if (!Z_IS_ZERO(z)){
			if(abs(z)< CAR_LENGTH)
				 move(0.0,-z,ALIGN_MOVE_SPEED,DURATION_10_S,0.005);
			else
				 move(0.0,-z,ALIGN_MOVE_SPEED);
            usleep(100*1000);
        }

        return 1;
    }

    int BackingUp::moveByCorner(float x,  float z, float angle)
    {
        PLOG_INFO(BACKING_UP_TAG,"moveByCorner() angle:%f (Deg:%f), x: %f, z: %f\n",angle,tf2Degrees(angle),x,z);
        if (PltConfig::getInstance()->isTrackModel())  /// Track model ...
        {
            // DeviceDefaultConfig cfg;
            float angle_ratio = mCfg.getAngleRatio();
            float mAngle = angle;
            PLOG_INFO(BACKING_UP_TAG,"Track is belt ... !! %f", angle_ratio);
            // Adjust angle,
            // Because the output of getHomeDistanceAndAngle() not correct and stable
            if(abs(angle) < M_PI * 20.0/180.0){
                mAngle = angle * angle_ratio;
            }
            else if (abs(angle) < M_PI*70.0/180.0){
                mAngle = angle * angle_ratio * 2.0/3.0;
            }
            else if (abs(angle) > M_PI_2){
                mAngle = M_PI_2;
            }

            if (abs(x)<= MOVE_DIST/8.0)
                return moveByCorner3(x,z,mAngle);
            else
                return moveByCorner2(x,z,mAngle);
        }

        if (!ANGLE_IS_ZERO(angle)){
            RETURN_IFCANCELD
            if (abs(angle)<2*MIN_ANGLE){
                if (angle<0){
                    angle -= MIN_ANGLE/2;
                }else{
                    angle += MIN_ANGLE/2;
                }
            }
            roll(angle,ALIGN_ROLL_SPEED);
            usleep(100*1000);
            RETURN_IFCANCELD
        }

        if (!X_IS_ZERO(x)){
            if (abs(x)<2*MIN_X_DIST){
                if (x<0){
                    x -= MIN_X_DIST/2;
                }else{
                    x += MIN_X_DIST/2;
                }
            }
            move(-x,0,ALIGN_MOVE_SPEED);
            usleep(100*1000);
            RETURN_IFCANCELD
        }

        if (!Z_IS_ZERO(z)){
            move(0,-z,ALIGN_MOVE_SPEED);
            usleep(100*1000);
        }

        return 1;
    }

    int BackingUp::moveByObj(float angle,  float z)
    {
        PLOG_INFO(BACKING_UP_TAG,"moveByObj() angle:%f (Deg:%f) , z:%f\n",angle,tf2Degrees(angle),z);
        if (PltConfig::getInstance()->isTrackModel())  /// Track model ...
		{
			PLOG_INFO(BACKING_UP_TAG,"Track is belt ... !!: %d",(abs(angle)<(M_PI/10.0)));
 				return moveByObj2(angle,z);
		}
        if (abs(angle)>BACK_ROLL_ERROR){
            PLOG_DEBUG(BACKING_UP_TAG,"moveByObj() roll:%f\n",angle);
			if(abs(angle <= M_PI/6.0 ))
				roll(angle,2.0,DURATION_15_S,DGREE_3_RAD/2.0);
			else
			if(abs(angle <= M_PI/3.0 ))
				roll(angle,2.0,DURATION_15_S,DGREE_2_RAD);
			else
				roll(angle,2.0);
            usleep(100*1000);
        }

        RETURN_IFCANCELD
        if (!Z_IS_ZERO(z)){
            PLOG_DEBUG(BACKING_UP_TAG,"moveByObj() move:%f\n",z);
            mAlgo.move(0, 0.618*z,ALIGN_MOVE_SPEED);
            usleep(100*1000);
        }

        return 2;
    }

    int BackingUp::moveByObj2(float angle,  float z)
    {
        float roll_factor = mCfg.getTrackTypeFloorRollFactor();
        PLOG_INFO(BACKING_UP_TAG,"moveByObj2() angle:%f (Deg:%f) , z:%f\n",angle,tf2Degrees(angle),z);
        if (abs(angle)>BACK_ROLL_ERROR){
            PLOG_DEBUG(BACKING_UP_TAG,"moveByObj2() roll:%f\n",angle);
			if((0 < angle ) && (angle <= M_PI/6.0 ))
			{
				roll(M_PI/3.0 ,ALIGN_ROLL_SPEED + roll_factor,DURATION_15_S,DGREE_3_RAD/2.0);
				usleep(100*1000);
				roll((angle - M_PI/3.0),ALIGN_ROLL_SPEED + roll_factor,DURATION_15_S,DGREE_3_RAD/2.0);
			}
			else
			if((0 > angle ) && (angle >= -M_PI/6.0 ))
			{
				roll(-M_PI/3.0 ,ALIGN_ROLL_SPEED + roll_factor,DURATION_15_S,DGREE_3_RAD/2.0);
				usleep(100*1000);
				roll((angle + M_PI/3.0),ALIGN_ROLL_SPEED + roll_factor,DURATION_15_S,DGREE_3_RAD/2.0);
			}
			else
			if(abs(angle <= M_PI/3.0 ))
				roll(angle,ALIGN_ROLL_SPEED + roll_factor,DURATION_15_S,DGREE_2_RAD);
			else
				roll(angle,2.0);
            usleep(100*1000);
        }

        RETURN_IFCANCELD
        if (!Z_IS_ZERO(z)){
            PLOG_DEBUG(BACKING_UP_TAG,"moveByObj2() move:%f\n",z);
            if(z > 0){
                mAlgo.move(0, std::min(z-ALIGN_DISTANCE, 0.618*z), ALIGN_MOVE_SPEED);
            } else  {
                mAlgo.move(0, z, ALIGN_MOVE_SPEED);
            }
            usleep(100*1000);
        }

        return 2;
    }

    bool BackingUp::canceled()
    {
        return mCanceled;
    }
    void BackingUp::updateStatus(int status)
    {
        if (canceled() &&
            (status != status::BACK_UP_INACTIVE &&
              status != status::BACK_UP_CANCEL)){
                return;
        }
        mStatus=status;
        mPub.pubStatus(&mStatus,1);

		PLOG_INFO(BACKING_UP_TAG,"publish backingup status(%d)\n", status);
        std_msgs::Int32 goHomeStatus;
        goHomeStatus.data = status;
        mStatusPub.publish(goHomeStatus);
    }

    void BackingUp::pubStatus(int status)
    {
        std_msgs::Int32 goHomeStatus;
        goHomeStatus.data = status;
        mStatusPub.publish(goHomeStatus);
    }

    bool BackingUp::cancel(nav_cancelRequest& req,nav_cancelResponse& res)
    {
        mCanceled = true;
        PLOG_INFO(BACKING_UP_TAG,"recv cancel_nav_backup topic\n");
        cancelBackingUp();
        return true;
    }

    bool BackingUp::cancelByCharged() {
        cancelBackingUp();
        return false;
    }

    void BackingUp::cancelBackingUp()
    {
        PLOG_INFO(BACKING_UP_TAG,"Cancel backingUp\n");
        std_msgs::Int32 goHomeStatus;
        goHomeStatus.data = status::BACK_UP_CANCEL;
        mStatusPub.publish(goHomeStatus);
        mAlgo.quitAlgo();
        updateStatus(status::BACK_UP_INACTIVE);
    }

    void BackingUp::onBatteryStatus(const statusConstPtr &s)
    {
        mCharging = s->status[2];
        if (mCharging &&
                (mStatus == status::BACK_UP_DETECT ||
                mStatus == status::BACK_UP_ALIGN))
        {
            PLOG_INFO(BACKING_UP_TAG,"charging: %d \n",mCharging);
            cancelBackingUp();
        }
    }

    int BackingUp::roll(float angle,float w,int timeout,float error)
    {
        if (PltConfig::getInstance()->isTrackModel())  /// Track model ...
			if (w<2.0) w += 2.0;			///1.5;                 ///roll for belt
        return mAlgo.roll(angle, w, timeout, error);
    }

    int BackingUp::move(float x,float y,float speed,int timeout,float error)
    {
        return mAlgo.move(x, y,speed, timeout, error);
    }

    void BackingUp::initEstimate()
    {
        if(!mOdomRelative){
            mOdomRelative=mGlobal.subscribe("MotorNode/baselink_odom_relative", 100, &BackingUp::odomRelativeCB,this);
        }

        if(!mTof){
            mTof=mGlobal.subscribe("SensorNode/tof",1,&BackingUp::tofDataCB,this);
        }
    }
    void BackingUp::deinitEstimate()
    {
        mOdomRelative.shutdown();
        mTof.shutdown();
    }

    void BackingUp::startAvoidObstacle()
    {
        mAdjusting=true;
        auto th=std::thread([this](){
            avoidObstacle();
            mAdjusting=false;
        });
        th.detach();
    }

    void BackingUp::avoidObstacle()
    {
        roll(-M_PI,2.0);
        RETURN_IFCANCELED
        mAlgo.move(0.0,NAV_OBSTACLE_FORWARD_DIST,0.2);
        RETURN_IFCANCELED
    }

    void BackingUp::odomRelativeCB(const nav_msgs::Odometry::ConstPtr& msg)
    {
        Eigen::Vector3d t(msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);
        float vx=msg->twist.twist.linear.x;
        float vy=msg->twist.twist.linear.y;
        float w=msg->twist.twist.angular.z;

        mCurPostion+=mCurPose*t;
        mCurPose=mCurPose*Eigen::Quaterniond(msg->pose.pose.orientation.w,msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z);

        TrackPoint point(msg->header.stamp.toNSec(),mCurPostion,mCurPose,vx,vy,w);
        mCurrent.push(point);
        doTrace(msg->header.stamp.toSec());
    }

    void BackingUp::tofDataCB(const sensor_msgs::RangeConstPtr &r)
    {
        if(r->range<NAV_OBSTACLE_MIN_DIST){
            mDetectObstacleCnt++;
        }else{
            mDetectObstacleCnt=0;
        }
    }

    void BackingUp::startRoll(double angle)
    {
        mAdjusting=true;
        auto th=std::thread([this,angle](){
            roll(angle,2.0);
            mAdjusting=false;
        });
        th.detach();
    }

    void BackingUp::doTrace(double stamp)
    {
        if(!mIsTracing){
            return;
        }
        if(mTrace.done()){
            stopTrace();
           return;
        }
        mLastTraceStamp=stamp;

        if(mAdjusting){
            return;
        }

        if(mDetectObstacleCnt>NAV_OBSTACLE_CNT){
            PLOG_DEBUG(BACKING_UP_TAG,"Detect obstacle");
            return;
        }

        mTrace.updatePose(mCurPostion,mCurPose);

        float vx,vy,w,roll;
        PLOG_DEBUG(BACKING_UP_TAG,"mTrace size:%d", mTrace.size());
        if(mTrace.traceOnce(vx,vy,w,roll)){
            //startRoll(roll);  //don't roll
        }else{
            geometry_msgs::Twist twist;
            twist.linear.x=vx;
            twist.linear.y=vy;
            twist.angular.z=w;
            mCmdVel.publish(twist);
        }
    }

    bool BackingUp::startTrace(bool reset)
    {
        if(reset){
            mCurPose=Eigen::Quaterniond(1.0,0.0,0.0,0.0);
            mCurPostion=Eigen::Vector3d(0.0,0.0,0.0);
            mCurrent.clear();
        }
        initEstimate();
        return mOdomRelative;
    }
    void BackingUp::stopTrace()
    {
        mIsTracing = false;
        deinitEstimate();
    }

    void differenceVector(vector<float> &data)
    {
        int i;
        for(i=1;i<(int)data.size();i++){
            data[i-1]=data[i]-data[i-1];
        }
        data.resize(i-1);
    }
    void lineFit(cv::Point2f* p,int n,int stride,double &k,double &b,bool tran)
    {
        double sumX(0.0),sumY(0.0),sumXY(0.0),sumXX(0.0),sumYY(0.0);
        double x,y;
        for(int i=0 ;i<n;i++)
        {
            if(!tran){
                x=p[i*stride].x;
                y=p[i*stride].y;
            }else{
                x=p[i*stride].y;
                y=p[i*stride].x;
            }
            sumX+=x;
            sumY+=y;
            sumXY+=x*y;
            sumXX+=x*x;
            sumYY+=y*y;
        }
        double sxy=sumXY-sumX*sumY/n;
        double sxx=sumXX-sumX*sumX/n;
        k=sxy/sxx;
        b=sumY/n-sumX/n*k;
    }
    void cvMatToEigenRotation(cv::Mat& mat,Eigen::Matrix3d& rotation)
    {
        for(int i=0;i<3;i++)
            for(int j=0;j<3;j++){
                rotation(i,j)=mat.at<double>(i,j);
            }
    }
    void poseToMotion(Eigen::Vector3d& pos,Eigen::Matrix3d& rotation,float& s,float& wBegin,float& wEnd)
    {
        Eigen::AngleAxisd rotationV(rotation);
        auto angle=rotationV.angle();
        auto axis=rotationV.axis();
        PLOG_DEBUG(BACKING_UP_TAG,"x:%f,z:%f,roll:[%f,%f,%f,%f]",pos.x(),pos.z(),angle,axis.x(),axis.y(),axis.z());
        PLOG_DEBUG(BACKING_UP_TAG,"acos(abs(axis.y()): %f, %f",acos(abs(0.127193)), acos(abs(-0.997895)));
        PLOG_DEBUG(BACKING_UP_TAG,"acos(abs(axis.y()): %f",acos(abs(axis.y())));

        if(abs(angle)>ALIGN_GOOD_ESTIMATE_ROLL && acos(abs(axis.y()) > ALIGN_GOOD_ESTIMATE_ANGLE)){//bad estimate
            PLOG_WARN(BACKING_UP_TAG,"bad angle estimate:x:%f,z:%f,roll:[%f,%f,%f,%f]",pos.x(),pos.z(),angle,axis.x(),axis.y(),axis.z());
            s=sqrt(pos.x()*pos.x()+pos.z()*pos.z());
            return;
        }
        if(abs(pos.x())<ALIGN_X_ERROR&& abs(pos.z())<ALIGN_Z_ERROR){
            s=0;
        }else{
            s=sqrt(pos.x()*pos.x()+pos.z()*pos.z());
            if(pos.z()>0){
                s=-s;
            }
        }

        if(axis.y()<0){
            angle=-angle;
        }
        if(s==0){
            wBegin=0;
            wEnd=angle;
        }else{
            if(pos.z()!=0){
                wEnd=atan(pos.x()/pos.z());
            }else{
                if(pos.x()>0){
                    wEnd=-M_PI/2;
                }else{
                    wEnd=M_PI/2;
                }
            }
            wBegin=angle-wEnd;
        }

        PLOG_DEBUG(BACKING_UP_TAG,"s:%f,wb:%f,we:%f\n",s,wBegin,wEnd);
    }
     /*
    base frame: x->righ,y->down,z->front
    imu frame: x->right,y->front,z->up
    base to came: baseR
    imu to base: imuR [
        1,0,0
        0,0,-1,
        0,1,0
    ]
    imu to came:(baseR*imuR)(-1) <update this matrix to vins_param.yaml:extrinsicRotation>
    */
    void calcBasePose(Eigen::Vector3d& pos,Eigen::Matrix3d& rot)
    {
        Eigen::Matrix3d baseR;
        Eigen::Vector3d baseT;
        //use macro  BACKING_UP_CALIBR to calibrate
        baseR<<0.9999997910906673, -7.757062183366373e-05, -0.0006417175550651816,
                        0.000272208947784179, 0.9509862392427677, 0.3092330814663691,
                        0.0005862771619266147, -0.3092331915459529, 0.9509860616882844;
        baseT<<-0.005243877716531838,-0.01568673974487856,-0.05983356995361024;
        Eigen::Matrix3d baseToWorldR=rot*baseR;
        Eigen::Vector3d baseToWorldT=rot*baseT+pos;
        rot=baseToWorldR;
        pos=baseToWorldT;
        std::cout<<"calcBasePose R:"<<rot<<std::endl<<"T:"<<pos<<std::endl;
    }

    bool calcCameraPoseEx(float z,cv::Mat &grey,Eigen::Vector3d& pos, cv::Rect &roi, float &xdist, float &zdist,float &angle)
    {
        cv::Size size(CHESSBAORD_COL-1,CHESSBAORD_ROW-1);
        vector<cv::Point2f> corners;
        int totalConers=size.width*size.height;
        cv::Mat roiGrey=grey(roi);
        cv::Mat clachedGrey;
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        clahe->apply(roiGrey,clachedGrey);

    #if SAVE_IMG
        char filenName[128];
        static int notFoundCnt = 0;
        static int foundCnt = 0;
    #endif
        PLOG_DEBUG(BACKING_UP_TAG,"cv::findChessboardCorners\n");
        if(!cv::findChessboardCorners(clachedGrey,size,corners)){
            PLOG_DEBUG(BACKING_UP_TAG,"cann't find corners!!!\n");
            #if SAVE_IMG
            sprintf(filenName, "/userdata/roller_eye/chessboard/err_%d.bmp", (notFoundCnt++)%20);
            cv::imwrite(filenName,clachedGrey);
            sprintf(filenName, "/userdata/roller_eye/chessboard/err_full_%d.bmp", (notFoundCnt++)%20);
            cv::imwrite(filenName,grey);
            #endif
            return false;
        }
        #if SAVE_IMG
        sprintf(filenName, "/userdata/roller_eye/chessboard/ok_%d.bmp", (foundCnt++)%20);
        cv::imwrite(filenName,clachedGrey);
        #endif
        if((int)corners.size()!=totalConers){
            PLOG_WARN(BACKING_UP_TAG,"corner count error!!!\n");
            return false;
        }

        cornerSubPix(roiGrey,corners,cv::Size(2, 2),cv::Size(-1,-1),cv::TermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,30,0.1));
        for(int i=0;i<(int)corners.size();i++){
           corners[i].x+=roi.x;
           corners[i].y+=roi.y;
        }
        vector<cv::Point3f> reals;
        float XOFFSET=(CHESSBAORD_COL-2)*CHESSBAORD_SIZE/2;
        float YOFFSET=(CHESSBAORD_ROW-2)*CHESSBAORD_SIZE/2;
        for(int i=0;i<CHESSBAORD_ROW-1;i++)
            for(int j=0;j<CHESSBAORD_COL-1;j++){
                reals.emplace_back(j*CHESSBAORD_SIZE-XOFFSET,i*CHESSBAORD_SIZE-YOFFSET,z);
            }

        cv::Mat camK=(cv::Mat_<double>(3,3)<<CAM_K_FX,0.0,CAM_K_CX,0.0,CAM_K_FY,CAM_K_CY,0.0,0.0,1.0);
        cv::Mat distro=(cv::Mat_<double>(4,1)<<DISTRO_K1,DISTRO_K2,DISTRO_P1,DISTRO_P2);
        cv::Mat r,t,R;
        solvePnP( reals, corners, camK, distro, r, t, false/*, cv::SOLVEPNP_EPNP */);
        cv::Rodrigues(r, R);
#ifdef  BACKING_UP_CALIBR
        std::cout<<"r:"<<R<<std::endl<<"t:"<<t<<std::endl;
#endif

        cv::Mat_<float> Tvec;
        t.convertTo(Tvec, CV_32F);   //平移向量

        angle =  atan2(R.at<double>(1, 0), R.at<double>(0, 0));

        Eigen::Matrix3f R_n;
        Eigen::Vector3f T_n;
        cv::cv2eigen(R, R_n);
        cv::cv2eigen(Tvec, T_n);
        Eigen::Vector3f P_oc;

        P_oc = -R_n.inverse()*T_n;
        std::cout<< "world coord" << P_oc << std::endl;
        xdist = P_oc[0];
        zdist = P_oc[2];

        std::cout<<"xdist 0: "<<xdist<<" zdist 0:"<<zdist<<" angle："<<angle<<endl;
        xdist -= sin(angle)*CAR_LENGTH;
        zdist = cos(angle)*zdist;
        zdist += ALIGN_DISTANCE;
        std::cout<<"xdist 1: "<<xdist<<" zdist 1:"<<zdist<<" angle："<<angle<<endl;

        Eigen::Vector3d trans;
        Eigen::Matrix3d rotation;

        cvMatToEigenRotation(R,rotation);
        Eigen::Matrix3d rot=rotation.transpose();//R^(-1)=R^(T)

        trans.x()=t.at<double>(0,0);
        trans.y()=t.at<double>(1,0);
        trans.z()=t.at<double>(2,0);
        pos=rot*trans*(-1);
        return true;
    }
}
