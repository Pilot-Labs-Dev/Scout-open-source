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
#define DETECT_MOVE_SPEED              (0.15)
#define DETECT_STOP_DISTANCE        (0.3)
#define DETECT_STOP_ERROR               (0.1)
#define DETECT_NOMALISE_DISTANCE    (0.3f)
#define DETECT_NORMALISE_HEIGHT   (0.264)  //the height/PIC_HEIGHT of charging pile while the disance between charge pile and  car is DETECT_NOMALISE_DISTANCE,a coarse value
#define CAR_LENGTH                                   (0.1)
// #define MIN_X_DIST (1.0e-2)
// #define MIN_Z_DIST (4.0e-2)
// #define MIN_ANGLE  ( 2.5e-2)

// #define X_IS_ZERO(x)                       (abs(x)<MIN_X_DIST)
// #define Z_IS_ZERO(x)                       (abs(x)<MIN_Z_DIST)
// #define ANGLE_IS_ZERO(x)            (abs(x)<MIN_ANGLE)

#define GOTO_IFCANCELD if(canceled()) { goto out;}
#define RETURN_IFCANCELD if(canceled()) { return 0;}

//BACKING 
#define BACK_ROLL_ERROR                      (1.5*M_PI/180) //1 degree
#define BACK_ROLL_SPEED                          1.0f
#define BACK_ROLL_ANGLE                       (175*M_PI/180)
#define BACK_ROLL_TIMEOUT                  (20000)

//
// #define ALIGN_IMG_WIDTH                         1280
// #define ALIGN_IMG_HEIGHT                        720
#define ALIGN_MAX_TIMES                        60
#define ALIGN_MAX_DETECT_FAIL_TIMES       100
#define ALIGN_ROLL_SPEED                          1.5f
#define ALIGN_MOVE_SPEED                          0.1f
//#define ALIGN_DISTANCE                          0.22
#define ALIGN_DISTANCE                          0.16
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

#define MOVE_DIST             0.4
#define VOL_AVG_TIMES   10
#define CHARGING_DETECT_TIMES   (30*60)

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
        mBatteryStatus = handle.subscribe("SensorNode/simple_battery_status",10,&BackingUp::onBatteryStatus,this);
        mStatusPub=  handle.advertise<std_msgs::Int32>("going_home_status",1);  
        mCancel=handle.advertiseService("nav_cancel",&BackingUp::cancel,this);
        mCmdVel=mGlobal.advertise<geometry_msgs::Twist>("cmd_vel",100);
        mAdjustClient = mGlobal.serviceClient<adjust_ligth>("/CoreNode/adjust_light");
        mNightGetClient = mGlobal.serviceClient<night_get>("CoreNode/night_get");
        updateStatus(status::BACK_UP_INACTIVE);

        mTimer=mGlobal.createTimer(ros::Duration(1),&BackingUp::timerCallback,this);
    }

    BackingUp::~BackingUp()
    {
        
    }
    void BackingUp::start()
    {
        mCanceled = false;
        mThread=std::thread(&BackingUp::backupLoop,this);
    }
    void BackingUp::stop()
    {
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
        if(0 == ret){
            std::cout << "left0: " << pos.left << ", " << pos.top << ", " << pos.right
                    <<", " << pos.bottom <<", "<<pos.width<<", "<<pos.height<<std::endl;
            return ret;
        }     
        return ret;
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
        mCanceled = false;
        updateStatus(status::BACK_UP_DETECT);
        mAlgo.resetQuitFlag();
        mBackupCounter = 0;        
        bool run=true;
        while(run && !canceled()){
            switch (mStatus)
            {
            case status::BACK_UP_DETECT:    
                adjustLight(1);
                doDetect();
                break;
            case status::BACK_UP_ALIGN:
                doAlign();
                break;
            case status::BACK_UP_BACK:
                adjustLight(1);
                doBackup();
                break;
            case status::BACK_UP_SUCCESS:
            case status::BACK_UP_FAIL:
            case status::BACK_UP_INACTIVE:
            case status::BACK_UP_CANCEL:
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
        std::cout << "left: " << pos.left << ", " << pos.top << ", " << pos.right
                    <<", " << pos.bottom <<", "<<pos.width<<", "<<pos.height<<std::endl;
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
        return bRet;
    }

    bool BackingUp::getHomeDistanceAndAngleByObj(AlgoOBjPos &pos,float &angle, float&z)
    {
        int centerX = (pos.left+pos.right)/2;
        int height   = pos.bottom-pos.top;
        int width    = pos.right-pos.left;


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
                return false;
            }

            if (centerX>=X_CENTER_LEFT &&
                   centerX<=X_CENTER_RIGHT){
                angle = 0;              
                break;
            }  
            if (centerX<X_CENTER_LEFT){
                angle = atan( A_COEFF * (1.0-float(centerX)/ X_CENTER));   
            }else{
                angle = -atan(A_COEFF * (float(centerX-X_CENTER))/ X_CENTER);     
            }        
        }while(0);

        if (angle<BACK_ROLL_ERROR && FLOAT_IS_ZERO(z)){
            return false;
        }
        return true;
    }

    int BackingUp::detectOnce()
    {
        int i;
        int timeout = 1.5*DETECT_TIMEOUT;
        for(i=0;i<DETECT_ROLL_TIME&&!canceled();i++){
             if(waitObj(HOME, timeout, mObjPos,1)==0 ){
                break;
            }    
            timeout = DETECT_TIMEOUT;
            if(roll(DETECT_ROLL_ANGLE,DETECT_ROLL_SPEED)<0){
                return BACKING_ERROR_MOVE;
            }
        }

        if(canceled()){
            return BACKING_DETECT_CANCEL;
        }
        if(i==DETECT_ROLL_TIME){
            return BACKING_DETECT_NO_HOME;
        } 

        return BACKING_STATUS_OK;  
    }

    void BackingUp::doDetect()
    {
        int ret;      
        mAlgo.waitObjBegin();
        for (int i=0; i<DETECT_COUNT; i++){
            ret = detect(i+1);          
            if (BACKING_DETECT_CANCEL == ret)  {
                goto out;
            }else if (BACKING_STATUS_OK == ret){
                updateStatus(status::BACK_UP_ALIGN);
                goto out;
            }
        }
        //updateStatus(status::BACK_UP_FAIL);
        updateStatus(status::BACK_UP_DETECT);

    out:
        mAlgo.waitObjEnd();
    }

    int BackingUp::detect(int times)
    {
        int ret=BACKING_DETECT_NO_HOME;
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
            for (int k=0; k<times; k++){                         
                ret=detectOnce();                  
                if(canceled()){
                    ret = BACKING_DETECT_CANCEL;
                    goto out;
                }
                if(ret==BACKING_STATUS_OK){
                    goto out;
                }else if(ret==BACKING_DETECT_NO_HOME){       
                    auto track=make_shared<TrackList>();
                    *track=tlst;
                    mTrace.setTrackList(track);
                    if (startTrace(true)){
                        mIsTracing = true;
                        while (mIsTracing){
                            if (canceled()){
                                stopTrace();
                                ret = BACKING_DETECT_CANCEL;
                                goto out;
                            }        
                            if(mDetectObstacleCnt>NAV_OBSTACLE_CNT){
                                mAdjusting=true;
                                AlgoOBjPos pos;
                                if(waitObj(HOME, DETECT_TIMEOUT, pos,0)==0){         
                                    mAdjusting=false;   
                                    stopTrace();    
                                    ret = BACKING_STATUS_OK;
                                    goto out;
                                }  
                                if (canceled()){
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
            }      
            roll(M_PI/3,DETECT_ROLL_SPEED);
        }       
        ret = BACKING_DETECT_NO_HOME;

    out:       
       return ret;
    }


     void BackingUp::doAlign()
    {
        mAlgo.waitObjBegin();
        
        float lastX = 0;
        float lastZ = 0;
        float angle = 0;

        bool bFound = false;
        bool bFirst = true;
        bool bBackup = false;
        bool bChess = false;
        bool bAdjust = false;
        int nMoveT = 0;
        bool bMoveLeft = false;
        do {
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
                    if (bChess){
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
                        continue;
                }

                if (nMoveT>0){
                    break;
                }
                if (!bMoveLeft){
                    moveByCorner(0.15,-0.05,0);
                    bMoveLeft = true;
                    nMoveT = 0;
                    continue;     
                }

            }else if (!bFound){        
                if (!bMoveLeft) {
                    break;
                }
                moveByCorner(-0.3,0,0);
                bMoveLeft = false;
            }

            if (2 == nMoveT){
                moveByObj(-angle/2,-lastZ/2);
            }else if (1==nMoveT){
                moveByCorner(-lastX/2, -lastZ/2, -angle/2);
            }else if (bMoveLeft){
                moveByCorner(-0.3,0,0);
                bMoveLeft = false;
            }
            GOTO_IFCANCELD
            bFound = false;
        }while(true);
        
        if (!bBackup){            
                updateStatus(status::BACK_UP_DETECT);
                goto out;
        }
        updateStatus(status::BACK_UP_BACK);
out:
        mAlgo.waitObjEnd();           
    }

    void BackingUp::doBackup()
    {
        RETURN_IFCANCELED

        mBackupRoll+=BACK_ROLL_ANGLE;
        if(mAlgo.rollEx(mAngle+BACK_ROLL_ANGLE,BACK_ROLL_SPEED,
                                        BACK_ROLL_TIMEOUT,BACK_ROLL_ERROR)<0){
            updateStatus(status::BACK_UP_FAIL);
            return;
        }
        RETURN_IFCANCELED
        sleep(1);
        RETURN_IFCANCELED
        mAlgo.action(0.0,-0.1,0.0,2200);

        RETURN_IFCANCELED
        if(mAlgo.connectToChargePile(1000)==0){    
            updateStatus(status::BACK_UP_SUCCESS);
        }else{
            RETURN_IFCANCELED
            if (++mBackupCounter < MAX_BACK_UP){
                mAlgo.move(0, 0.8,ALIGN_MOVE_SPEED);
                RETURN_IFCANCELED
                roll(M_PI,ALIGN_ROLL_SPEED);
                updateStatus(status::BACK_UP_ALIGN);
            }else{
                updateStatus(status::BACK_UP_FAIL);
            }
        }
    }

    bool BackingUp::canBackup(float x, float z, float angle)
    {

        if(Z_IS_ZERO(z) && 
              X_IS_ZERO(x) && 
             ANGLE_IS_ZERO(angle)){
                return true;
        }

        return false;
    }

   int BackingUp::moveByCorner(float x,  float z, float angle)
    { 
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
        if (abs(angle)>BACK_ROLL_ERROR){  
            roll(angle,2.0);
            usleep(100*1000);
        }

        RETURN_IFCANCELD
        if (!Z_IS_ZERO(z)){   
            mAlgo.move(0, 0.618*z,ALIGN_MOVE_SPEED);
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
    
    void  BackingUp::cancelCallback(const std_msgs::Int32::ConstPtr& msg)
    {
        mCanceled = true;   
        std_msgs::Int32 goHomeStatus;
        goHomeStatus.data = status::BACK_UP_CANCEL;
        mStatusPub.publish(goHomeStatus);
        mAlgo.quitAlgo();
        updateStatus(status::BACK_UP_INACTIVE);     
    }

    bool BackingUp::cancel(nav_cancelRequest& req,nav_cancelResponse& res)
    {
        mCanceled = true;  
        std_msgs::Int32 goHomeStatus;
        goHomeStatus.data = status::BACK_UP_CANCEL;
        mStatusPub.publish(goHomeStatus);   
        mAlgo.quitAlgo();
        updateStatus(status::BACK_UP_INACTIVE);
        return true;
    }
    
    void BackingUp::onBatteryStatus(const statusConstPtr &s)
    {
        if (s->status[2]){      
            //updateStatus(status::BACK_UP_SUCCESS);
        }
    }

    int BackingUp::roll(float angle,float w,int timeout,float error)
    {
        //return mAlgo.rollEx(angle, w, timeout, error);
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
            //startAvoidObstacle();   
            return;
        }

        mTrace.updatePose(mCurPostion,mCurPose);

        float vx,vy,w,roll;

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


        if(abs(angle)>ALIGN_GOOD_ESTIMATE_ROLL && acos(abs(axis.y()) > ALIGN_GOOD_ESTIMATE_ANGLE)){
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

        if(!cv::findChessboardCorners(clachedGrey,size,corners)){
            return false;
        }

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

        cv::Mat_<float> Tvec;
        t.convertTo(Tvec, CV_32F);

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

        xdist -= sin(angle)*CAR_LENGTH;
        zdist = cos(angle)*zdist;
        zdist += ALIGN_DISTANCE;

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