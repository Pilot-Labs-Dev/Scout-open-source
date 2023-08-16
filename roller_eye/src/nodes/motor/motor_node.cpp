#include<math.h>
#include<thread>
#include<mutex>
#include<unistd.h>
#include<fstream>
#include<opencv2/opencv.hpp>
#include"ros/ros.h"
#include"std_msgs/String.h"
#include"geometry_msgs/Twist.h"
#include"geometry_msgs/PoseStamped.h"
#include"nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include"sensor_msgs/Imu.h"
#include"sensor_msgs/Range.h"
#include"roller_eye/motor.h"
#include"roller_eye/plt_assert.h"
#include"roller_eye/data_publisher.h"
#include"roller_eye/plt_config.h"
#include"roller_eye/util_class.h"
#include"roller_eye/system_define.h"
#include"eigen3/Eigen/Core"
#include"roller_eye/util_class.h"
#include"roller_eye/status.h"
#include"std_msgs/Int32.h"

#include "roller_eye/param_utils.h"

#include "roller_eye/vio_start.h"
#include "roller_eye/vio_stop.h"
#include "roller_eye/getOdom.h"

#include "roller_eye/enable_vio.h"
#include "zlog.h"
#include "roller_eye/imu_patrol_calib.h"
#include "roller_eye/getimu_patrolcalib_status.h"

#define MOTOR_NODE_TAG  "MotorNode"

#define W_TO_F               (1.0/(MACC_PI*MACC_ROLL_DIAMETER))

#define ENALBE_DYNAMIC_ADJUST
#define STOP_VALUE                           0.00001f
#define ZERO_RAD                                0.0001f
#define ADJUST_TIME                         0.8
#define PID_P_EFFICIENT                  0.2

#define ENABLE_IMU_Z_ROLL

using namespace roller_eye;
using namespace std;

const static float eX=W_TO_F;
const static float eY=W_TO_F;
const static float eZ=(MACC_CAR_WIDTH/2+MACC_CAR_HEIGHT/2)*W_TO_F;

class InitDZLog{
    public:
    InitDZLog()
    {
    	log_t arg = {
			confpath:	"/var/roller_eye/config/log/" MOTOR_NODE_TAG ".cfg",
			levelpath:	"/var/roller_eye/config/log/log.level",
			logpath:	"/var/log/" MOTOR_NODE_TAG ".log",
			cname:		MOTOR_NODE_TAG
		};
		if(0 != dzlogInit(&arg,2)){
			printf("%s log int error.\r\n",MOTOR_NODE_TAG);
		}

    }
    ~InitDZLog()
    {
		dzlogfInit();
    }
} ;

class ScalarKalmanFilter{
public:
    ScalarKalmanFilter():
    mValue(0.0),
    mSigma2(0.0)
    {

    }
    ScalarKalmanFilter(double v):
    mValue(v),
    mSigma2(0.0)
    {
    }
    void predict(double delta,double sigma)
    {
        mValue+=delta;
        mSigma2+=sigma*sigma;
    }
    void update(double value,double sigma)
    {
        if(abs(sigma)<STOP_VALUE){
            mValue=value;
            mSigma2=0;
        }else{
            double err=value-mValue;
            double sigma2=sigma*sigma;
            double k=mSigma2/(mSigma2+sigma2);
            mValue+=k*err;
            mSigma2*=(1-k);
        }
    }
    double getValue()
    {
        return mValue;
    }
private:
    double mValue;
    double mSigma2;
};
class MotorPosePublisher{
public:
    MotorPosePublisher():
    mEnabled(false),
    mLocal("~"),
    mGlobal(""),
    W_ACCEL(10.0)// rad/s^2
    {
        DeviceDefaultConfig cfg;
        ACC_SIGMA=cfg.getMotorFilterAccSigma();
        UPDATE_SIGMA=cfg.getMotorFilterUpdateSigma();
        loadTune();
        resetPoseInfo();
    }
    ~MotorPosePublisher()
    {
    }

    void start()
    {
        resetPoseInfo();
        mEnabled=true;
    }

    void stop()
    {
        mEnabled=false;
    }
    int setData(nav_msgs::Odometry& odom)
    {
        odom.header.stamp=ros::Time::now();
        odom.header.frame_id="world";
        odom.child_frame_id="base_link";
        lock_guard<mutex> lock(mPoseMutex);
        updateTranslate();
        odom.pose.pose.position.x=mSx;
        odom.pose.pose.position.y=mSy;
        odom.pose.pose.position.z=0.0;
        #ifdef ENABLE_IMU_Z_ROLL
        odom.pose.pose.orientation=tf::createQuaternionMsgFromYaw(mZrolled); //use imu info
        #else
        odom.pose.pose.orientation=tf::createQuaternionMsgFromYaw(mSw); //use control info
        #endif
        odom.twist.twist.linear.x=mPreVx;
        odom.twist.twist.linear.y=mPreVy;
        odom.twist.twist.angular.z=mPreW;
        mZrolled=mSx=mSy=mSw=0.0;

        return 0;
    }
    void updateCmdVel(float vx,float vy,float w)
    {
        if(!mEnabled){
            return;
        }

        lock_guard<mutex> lock(mPoseMutex);
        mVx.update(mPreVx,UPDATE_SIGMA*mPreVx);
        mVy.update(mPreVy,UPDATE_SIGMA*mPreVy);
        mPreVtime=updateTranslate();
        getRealSpeed(vx,vy,w);

        mPreVx=vx;
        mPreVy=vy;
        mWdelta=w-mPreW;
        mPreW=w;
    }
    double accelKernel(double a)
    {
        if(abs(a)>1.0){
            return a;
        }
        return (a*a*a);
    }
    void updateMeasurement(double ax,double ay,double wz,double dt,bool stopped)
    {
        if(!mEnabled){
            return;
        }
        lock_guard<mutex> lock(mPoseMutex);
        if(stopped){
            mStopTime+=dt;
            if(mStopTime>0.2){
                mVx=0.0;
                mVy=0.0;
                mZrolled=0.0;
            }else{
                double angle=wz*dt;
                mZrolled+=angle;
                mVx.predict(accelKernel(ax)*dt,ACC_SIGMA);
                mVy.predict(accelKernel(ay)*dt,ACC_SIGMA);
                updateTranslate();
            }
        }else{
            mXStopTime = 0.0;
            mStopTime=0.0;
            double angle=wz*dt;
            mZrolled+=angle;
            mVx.predict(accelKernel(ax)*dt,ACC_SIGMA);
            mVy.predict(accelKernel(ay)*dt,ACC_SIGMA);
            updateTranslate();
        }
    }

    void updateMeasurement(double ax,double ay,double wz,double dt)
    {
        if(!mEnabled){
            return;
        }
        lock_guard<mutex> lock(mPoseMutex);
        mXStopTime+=dt;
        double angle=wz*dt;
        mZrolled+=angle;
        if(mXStopTime>0.2){
            mVx=0.0;
        }else{
            mVx.predict(accelKernel(ax)*dt,ACC_SIGMA);
        }
        mVy.predict(accelKernel(ay)*dt,ACC_SIGMA);
        updateTranslate();
    }

    void resetPoseInfo()
    {
        lock_guard<mutex> lock(mPoseMutex);
        mPreVtime=-1.0;
        mVx=0.0;
        mVy=0.0;
        mPreVx=mPreVy=mPreW=mWdelta=0.0;
        mSx=mSy=mSw=0.0;
        mZrolled=0.0;
        mStopTime=0.0;
        mXStopTime = 0.0;
    }

private:
    double updateTranslate()
    {
        double curTime=ros::Time::now().toSec();
        if(mPreVtime > 0){
            double t=curTime-mPreVtime;
            mSx+=mVx.getValue()*t;
            mSy+=mVy.getValue()*t;
#ifndef ENABLE_IMU_Z_ROLL
            double wt=mWdelta/W_ACCEL;
            if(wt>t){
                double deltaW=t*W_ACCEL;
                mSw+=(mPreW-deltaW/2)*t;
                mWdelta-=deltaW;
            }else{
                mSw+=(mPreW*t-wt*mWdelta/2);
                mWdelta=0.0;
            }
#endif
            mPreVtime=curTime;
        }

        return curTime;
    }

    void getRealSpeed(float &vx,float &vy,float &w)
    {
        if(mTunes.empty()){
            return;
        }
        cv::Mat coeffi=(cv::Mat_<float>(MOTOR_NUM, 3) <<
                        -eX,eY,eZ,
                        eX,eY,-eZ,
                        eX,eY,eZ,
                        -eX,eY,-eZ);
        cv::Mat speed=(cv::Mat_<float>(3, 1)<<vx,vy,w);
        cv::Mat  motorW=coeffi*speed;

        for(int i=0;i<MOTOR_NUM;i++){
            tuneMap(motorW.at<float>(i,0));
        }

        cv::Mat real;
        if(cv::solve(coeffi,motorW,real,CV_SVD)){
            vx=real.at<float>(0,0);
            vy=real.at<float>(1,0);
            w=real.at<float>(2,0);
        }else{
            ROS_WARN("solve speed fail");
        }
    }
    void tuneMap(float &value)
    {
        float sign=value>0?1:-1;
        float pos=value*sign/mTuneDelta;
        int idx=(int)pos;
        if(idx>(int)mTunes.size()-2){
            return ;
        }
        value = (mTunes[idx]*(idx+1-pos)+mTunes[idx+1]*(pos-idx))*sign;
    }
    void loadTune()
    {
        float exp,real;
        FILE *tuneFile=fopen(ROLLER_EYE_CONFIG_BASE"motor_tune","r");
        if(tuneFile==NULL){
            ROS_WARN("no motor tune files");
            return;
        }
        if(fscanf(tuneFile,"%f\n",&mTuneDelta)!=1){
            ROS_WARN("tune files  header error");
            goto out;
        }
        while(!feof(tuneFile)){
            if(fscanf(tuneFile,"%f %f\n",&exp,&real)!=2){
                ROS_WARN("tune files data error");
                mTunes.clear();
                goto out;
            }
            mTunes.push_back(real);
        }
        for(auto t:mTunes){
            ROS_INFO("%f",t);
        }
    out:
        fclose(tuneFile);
    }

    bool IsEnabled()
    {
        return mEnabled;
    }

    bool mEnabled;
    mutex mPoseMutex;
    double mPreVtime;
    ScalarKalmanFilter mVx;
    ScalarKalmanFilter mVy;
    float mPreVx;
    float mPreVy;
    float mPreW;
    float mWdelta;
    float mSx;
    float mSy;
    float mSw;
    double mZrolled;
    double mStopTime;
    double mXStopTime;

    float mTuneDelta;
    vector<float> mTunes;

    float  W_ACCEL;
    float  ACC_SIGMA;
    float UPDATE_SIGMA;
    ros::NodeHandle mLocal;
    ros::NodeHandle mGlobal;
};

class VioPosePublisher{
public:
    VioPosePublisher():
    mGlobal(""),
    mBVioStart(false)
    {
        mStart = mGlobal.serviceClient<vio_start>("/vio/start");
        mStop = mGlobal.serviceClient<vio_stop>("/vio/stop");
        mGetOdom = mGlobal.serviceClient<getOdom>("/vio/getOdom");
    }
    ~VioPosePublisher()
    {
    }

    void start()
    {
        vio_start start;
        start.request.on = 1;
        if (mStart.call(start)){
            mBVioStart = true;
        }else{
            ROS_ERROR("vio start error!\n");
        }
    }

    void stop()
    {
        vio_stop stop;
        stop.request.off=1;
        if (!mStop.call(stop)){
            ROS_ERROR("vio stop error!\n");
        }
        mBVioStart = false;
    }
    int setData(nav_msgs::Odometry& odom)
    {
        odom.header.stamp=ros::Time::now();
        odom.header.frame_id="world";
        odom.child_frame_id="base_link";
        if (mBVioStart){
            getOdom vio_odom;
            vio_odom.request.get=1;
            if (!mGetOdom.call(vio_odom)){
                ROS_ERROR("get vio_odom error!\n");
                return -1;
            }else{
                ROS_ERROR("get vio_odom ok!\n");
                odom.header.seq=vio_odom.response.header.seq;
                odom.pose.pose.position.x=vio_odom.response.x;
                odom.pose.pose.position.y=vio_odom.response.y;
                odom.pose.pose.position.z=0.0;

                ROS_DEBUG("vio get odom: x:%f, y:%f, yaw=%f",odom.pose.pose.position.x, odom.pose.pose.position.y, vio_odom.response.yaw);
                odom.pose.pose.orientation=tf::createQuaternionMsgFromYaw(vio_odom.response.yaw);
            }
        }
        return 0;
    }

private:
    bool mBVioStart;
    ros::NodeHandle mGlobal;
    ros::ServiceClient mStart;
    ros::ServiceClient mStop;
    ros::ServiceClient mGetOdom;
};

class MotorDriver{
public:
    MotorDriver():
    mHandle(""),
    mMotor(createMotor()),
    mRunning(true),
    mPowerOn(false),
    mStopped(true),
    mKeepMove(false),
    mMotorEnable(true),
    mEnableDynamicAdjust(mCfg.getMotorAdjust()),
    STOP_TIME_THRESHOLD(mCfg.getMotorStopDelay()),
    MOTOR_POWEROFF_TIMEOUT(mCfg.getMotorPoweroffDelay()),
    mMotorPowerOffTimeout(MOTOR_POWEROFF_TIMEOUT),
    mCalibra(0.0,0.0,0.0),
    mDirFixMode(mCfg.getMotorFixDir()),
    mDirAngle(0.0),
    mPosePub(new MotorPosePublisher()),
    GYRO_CALIBRA_THRESHOLD(0.05),
    ACCEL_CALIBRA_THRESHOLD(0.2),
    IMU_CALIBRA_CNT(500),
    mChargingStatus(-1),
    mCmdVelCnt(1000),
    OBSTACLE_MIN_DISTANCE(mCfg.getObstacleMinDistance()),
    SLOW_SPEED_DISTANCE(mCfg.getSlowSpeedDistance()),
    OBSTACLE_SPEED(mCfg.getObstacleSpeed()),
    mDistance(DBL_MAX),
    mMinDisCnt(0),
    mDoubleMinDistCnt(0),
    mAvgTof(MAX_VALID_TOF_DIST),
    mTofAvgCnt(TOF_AVG_COUNT),
    mBPatrolCalib(false),
    mVx(0),
    mVy(0),
    mVw(0)
    {
        resetAdjustParam();
        resetAutoIMUCalibra();
        setupMotor();
        mCmdVel=mHandle.subscribe("cmd_vel", 1, &MotorDriver::volecityCallback,this);
        mForceCmdVel=mHandle.subscribe("cmd_vel_force", 1, &MotorDriver::velocityForceCallback,this);

        mCmdVel2=mHandle.subscribe("cmd_vel2", 100, &MotorDriver::volecityCallback2,this);
        mCmdVel3=mHandle.subscribe("cmd_vel3", 100, &MotorDriver::volecityCallback3,this);
        mCmdVel4=mHandle.subscribe("cmd_vel4", 100, &MotorDriver::volecityCallback4,this);
        mCmdVel8003=mHandle.subscribe("cmd_vel8003", 100, &MotorDriver::volecityCallback8003,this);
        mBattery=mHandle.subscribe("SensorNode/simple_battery_status",5,&MotorDriver::batteryStatusCallback,this);
        mNav=mHandle.subscribe("NavPathNode/status",5,&MotorDriver::navStatusCallback,this);
        mAlgBack=mHandle.subscribe("CoreNode/going_home_status",5,&MotorDriver::goingHomeStatus,this);
        mImuCalib=mHandle.advertiseService("imu_patrol_calib",&MotorDriver::imuPatrolCalib,this);
        mImuPatrolCalibStatus=mHandle.advertiseService("getimu_patrolcalib_status",&MotorDriver::imuPatrolCalibStatus,this);
    }
    ~MotorDriver()
    {
       mCmdVel.shutdown();
       mRunning=false;
       mMotorEnable=false;
       mCondi.notify_one();
       mMotorDaemon.join();
    }
    void enable(bool flag)
    {
        mMotorEnable=flag;
    }
    void enable_adjust(bool flag)
    {
        mEnableDynamicAdjust=flag;
    }
    void enable_dir_fixed(bool flag)
    {
        mDirFixMode=flag;
        mDirAngle=0.0;
    }
    shared_ptr<MotorPosePublisher> getPosePublisher()
    {
        return mPosePub;
    }
private:
    void start_motor_daemon()
    {
        mMotorDaemon=thread([this](){
            while(true){
                unique_lock<mutex> lock(mMutex);
                mCondi.wait(lock,[this]()->bool{
                    return mPowerOn||!mRunning;
                });
                if(!mRunning){
                    break;
                }

                mKeepMove=false;
                lock.unlock();
                usleep(STOP_TIME_THRESHOLD);
                lock.lock();
                if(!mKeepMove){
                    setX_Y_Wz(0,0,0);
                    mMotorPowerOffTimeout-=STOP_TIME_THRESHOLD;
                    if(mMotorPowerOffTimeout<=0){
                        plt_assert(motor_stop()==0);
                        mPowerOn=false;
                    }
                }else{
                   mMotorPowerOffTimeout= MOTOR_POWEROFF_TIMEOUT;
                }
            }
        });
    }
    void keep_move()
    {
        unique_lock<mutex> lock(mMutex);
        mKeepMove=true;
        if(!mPowerOn){
            plt_assert(motor_start()==0);
            mPowerOn=true;
            mMotorPowerOffTimeout=MOTOR_POWEROFF_TIMEOUT;
            mCondi.notify_one();
        }
    }
    struct MotorCtl{
        int idx;
        float speed;
        bool p;
    };

    /*
    motor idx

                        front
                    1               0
    left                                    right
                    3               2
                        back
    Y
    ^
    |
    |
    |
    |
    |------------------------------> X

    Move in X, direction of 4 motors,  1 is Positive ,-1 is nagetive
                    1               -1

                    -1               1
    Move in Y
                    1               1

                    1                1
    Rotate of Z
                    -1              1

                    -1              1
    matrix:
    motor0          -1   1      1                 eX
    motor1           1    1    -1
    motor2           1    1      1       *       eY
    motor3          -1    1     -1               ez
    */
    void setX_Y_Wz(float x,float y,float z)
    {
        PLOG_DEBUG(MOTOR_NODE_TAG, "setX_Y_Wz: x: %f, y: %f, z: %f", x, y, z);
        vector<MotorCtl> pars(MOTOR_NUM);
        float speed;
        // float weight[MOTOR_NUM][3]={
        //     {-eX,eY,eZ},
        //     {eX,eY,-eZ},
        //     {eX,eY,eZ},
        //     {-eX,eY,-eZ}
        // };
        std::vector<vector<float>>  weight={
            {-eX,eY,eZ},
            {eX,eY,-eZ},
            {eX,eY,eZ},
            {-eX,eY,-eZ}
        };
        switch (mMotor->GetDriverType()){
            case MotorDriverType::FS8003:{
                    std::vector<vector<float>>  weightFS8003={
                        {-eX,eY,eZ},
                        {eX,eY,eZ},
                        {eX,eY,-eZ},
                        {-eX,eY,-eZ}
                    };
                    weight = weightFS8003;
            }
            break;
            default:
            break;
        }

        if (PltConfig::getInstance()->isTrackModel()){  /// Track model ...
            mMotor->convertSpeedTrackModel(x,y);
        } else {
            mMotor->convertSpeed(x,y);
        }

        for(int i=0;i<MOTOR_NUM;i++){
            pars[i].idx=i;
            speed=x*weight[i][0]+y*weight[i][1]+z*weight[i][2];
            if(speed>=0){
                pars[i].speed=speed;
                pars[i].p=true;
            }else{
                pars[i].speed=-speed;
                pars[i].p=false;
            }
        }

        for(auto& par : pars){
            mMotor->setParam(par.idx,par.speed,par.p?MOTOR_STATUS_POSITIVE:MOTOR_STATUS_NEGTIVE);
        }
        TimeCost c;
        mMotor->flush();
        mStopped=isStopCmd(x,y,z);
    }
    inline float checkVolecity(float v,float vMax)
    {
            v=std::max(-vMax,v);
            v=std::min(vMax,v);
            return v;
    }
    void avoidObstacle(float &x,float &y,float &wz)
    {
        double ymax=MACC_MAX_SPEED_Y;

        if(mAvgTof<OBSTACLE_MIN_DISTANCE){
            ymax=0.0;
        }else if( mAvgTof<SLOW_SPEED_DISTANCE ){
            ymax=OBSTACLE_SPEED+(mDistance-OBSTACLE_MIN_DISTANCE)/OBSTACLE_MIN_DISTANCE*OBSTACLE_SPEED;
        }

        if(y>ymax){
            PLOG_DEBUG(MOTOR_TAG, "detect obstacle,slow speed  ymax=%f  mAvgTof:%f!!!",ymax, mAvgTof);
            y=ymax;
        }
    }
    void volecityCallback(const geometry_msgs::TwistConstPtr& msg)
    {
        PLOG_DEBUG(MOTOR_NODE_TAG, "recieve vel: x=%f,y=%f,w=%f. motorEnable: %d",msg->linear.x,msg->linear.y,msg->angular.z, mMotorEnable);
        if(!mMotorEnable){
            ROS_DEBUG("recieve vel0,x=%f,y=%f,w=%f",msg->linear.x,msg->linear.y,msg->angular.z);
            return ;
        }
        keep_move();

        float x=checkVolecity(msg->linear.x,MACC_MAX_SPEED_X);
        float y=checkVolecity(msg->linear.y,MACC_MAX_SPEED_Y);
        float wz=checkVolecity(msg->angular.z,MACC_MAX_SPEED_W);


        avoidObstacle(x,y,wz);

        fixMotorDir(x,y);
        adjust(x,y,wz);

        float err=mAngleError/ADJUST_TIME-mAngleVelError;
        if(abs(err)>1.0){
            err=(err>0?1.0:-1.0);
        }

        PLOG_DEBUG(MOTOR_NODE_TAG, "x:%f, y:%f, wz:%f, err:%f wz-err:%f, mStopped=%d",x,y,wz, err, wz-err, mStopped);
        setX_Y_Wz(x,y,wz-err);

        mPosePub->updateCmdVel(x,y,wz);
        mVx =x;
        mVy = y;
        mVw =wz;
    }

    /*
    *  Control the scout without fixing by imu data and tof data
    */
    void velocityForceCallback(const geometry_msgs::TwistConstPtr& msg) {
        PLOG_DEBUG(MOTOR_NODE_TAG, "cmd_vel_force recieved vel: x=%f,y=%f,w=%f. motorEnable: %d",msg->linear.x,msg->linear.y,msg->angular.z, mMotorEnable);
        if(!mMotorEnable){
            PLOG_ERROR(MOTOR_NODE_TAG, "cmd_vel_force recieved vel0,x=%f,y=%f,w=%f but motorEnable not true",msg->linear.x,msg->linear.y,msg->angular.z);
            return ;
        }
        keep_move();
        float x=checkVolecity(msg->linear.x,MACC_MAX_SPEED_X);
        float y=checkVolecity(msg->linear.y,MACC_MAX_SPEED_Y);
        float wz=checkVolecity(msg->angular.z,MACC_MAX_SPEED_W);
        setX_Y_Wz(x,y,wz);

        mPosePub->updateCmdVel(x,y,wz);
        mVx =x;
        mVy = y;
        mVw =wz;
    }

    void volecityCallback2(const geometry_msgs::TwistConstPtr& msg)
    {
        ROS_DEBUG("volecityCallback2 vel,x=%f,y=%f,w=%f",msg->linear.x,msg->linear.y,msg->angular.z);


        keep_move();

        vector<int> vDir;
        vector<int> vSpeed;
        for(int i=0;i<MOTOR_NUM;i++){
            vDir.push_back(0);
        }
        // vDir[2] = 1;
        // vDir[3] = 0;
        vDir[0] = 0;
        vDir[1] = 1;
        for(int i=0;i<MOTOR_NUM;i++){
            vSpeed.push_back(0);
        }
        int speed = static_cast<int>(200*msg->linear.y);
        if (abs(speed)>100){
            speed = 100;
        }
        vSpeed[0] = speed;
        vSpeed[1] = speed;

        mMotor->drive(vDir, vSpeed);
    }

    void volecityCallback3(const geometry_msgs::TwistConstPtr& msg)
    {
        ROS_DEBUG("volecityCallback3 vel,x=%f,y=%f,w=%f",msg->linear.x,msg->linear.y,msg->angular.z);


        keep_move();

        vector<int> vDir;
        vector<int> vSpeed;
        for(int i=0;i<MOTOR_NUM;i++){
            vDir.push_back(0);
        }

        if (msg->linear.y>0){
            vDir[0] = 1;
            vDir[2] = 1;
            vDir[1] = 1;
            vDir[3] = 1;
        }else if (msg->linear.y<0){
            vDir[0] = 0;
            vDir[2] = 0;
            vDir[1] = 0;
            vDir[3] = 0;
        }
        // vDir[0] = 0;
        // vDir[1] = 1;
        for(int i=0;i<MOTOR_NUM;i++){
            vSpeed.push_back(0);
        }
        int speed =abs( static_cast<int>(200*msg->linear.y));
        if (speed>100){
            speed = 100;
        }
        vSpeed[0] = speed;
        vSpeed[1] = speed;
        vSpeed[2] = speed;
        vSpeed[3] = speed;

        mMotor->drive(vDir, vSpeed);
    }

    void volecityCallback4(const geometry_msgs::TwistConstPtr& msg)
    {
        ROS_DEBUG("volecityCallback3 vel,x=%f,y=%f,w=%f",msg->linear.x,msg->linear.y,msg->angular.z);


        keep_move();

        vector<int> vDir;
        vector<int> vSpeed;
        for(int i=0;i<MOTOR_NUM;i++){
            vDir.push_back(0);
            vSpeed.push_back(0);
        }

        int idx = static_cast<int> (msg->linear.x);
        if (msg->linear.y>0){
            vDir[idx] = 1;
        }else if (msg->linear.y<0){
            vDir[idx] = 0;
        }

        int speed =abs( static_cast<int>(200*msg->linear.y));
        if (speed>100){
            speed = 100;
        }
        vSpeed[idx] = speed;
        mMotor->drive(vDir, vSpeed);
    }

    void volecityCallback8003(const geometry_msgs::TwistConstPtr& msg)
    {
        int idx = (int)msg->linear.x;
        float speed = msg->linear.y;
        mMotor->driveF8003(idx, speed);
    }

    bool isStopCmd(float x,float y,float wz)
    {
        return (abs(x)<STOP_VALUE && abs(y)<STOP_VALUE&& abs(wz)<STOP_VALUE);
    }
    void adjust(float x,float y,float wz)
    {
#ifdef ENALBE_DYNAMIC_ADJUST
        if(!mEnableDynamicAdjust||abs(wz)>ZERO_RAD||abs(mPreAngleVel)>ZERO_RAD||isStopCmd(x,y,wz)){
            mAngleError=0;
            mAngleVelError=0;
            mSkipAjustCnt=0;
        }else{
            if(++mSkipAjustCnt<10){
                mAngleError=0;
            }else{
                if(mIMUcnt!=0){
                    mAngleVelError+=(mPreAngleVel-mAngleVelSum/mIMUcnt)*PID_P_EFFICIENT;
                }
            }
        }
        mIMUcnt=0;
        mAngleVelSum=0;
        mPreAngleVel=wz;
#endif
    }
    void imuCallback(const sensor_msgs::ImuConstPtr& msg)
    {
        static double stoptime = 0;
        static double prevtime = 0;
        static double prevStopTime = -1;
        double cur=msg->header.stamp.toSec();

        double x=msg->linear_acceleration.x-mCalibra.x();
        double y=msg->linear_acceleration.y-mCalibra.y();
        double w=msg->angular_velocity.z-mCalibra.z();

        if (mStopped){
                stoptime +=(cur-mPreTime);
        }else{
                stoptime = 0;
        }


        mIMUcnt++;
        mAngleVelSum+=w;
        // mlpWZ = 0.8*w+0.2*mlpWZ;
        // w = mlpWZ;
        if(mPreTime>0){
            double t=(cur-mPreTime);
            double ax=(x+mPreIMU.x())/2;
            double ay=(y+mPreIMU.y())/2;
            double wz=(w+mPreIMU.z())/2;
            double delta=wz*t;
            mAngleError+=delta;
            mDirAngle+=delta;

            mPosePub->updateMeasurement(ax,ay,wz,t,mStopped);
        }
        mPreTime=cur;
        mPreIMU=Eigen::Vector3d(x,y,w);
        if (mStopped && stoptime>0.5){
#ifndef	SENSOR_IMU_USE_CALIBRA
            autoCalibraIMU(x,y,w);
#endif
        }
    }
    int motor_start()
    {
#ifndef SENSOR_IMU_USE_CALIBRA
        loadCalibra();
#endif
        resetAutoIMUCalibra();
#ifdef ENALBE_DYNAMIC_ADJUST
        resetAdjustParam();
        if( !mIMU){
            mIMU=mHandle.subscribe("SensorNode/imu", 1000, &MotorDriver::imuCallback,this);
        }
#endif
        return mMotor->powerON();
    }
    int motor_stop()
    {
        int ret= mMotor->powerOFF();
#ifdef ENALBE_DYNAMIC_ADJUST
        if(mIMU){
            mIMU.shutdown();
        }
#endif
        mVx = 0;
        mVy = 0;
        mVw = 0;
        mPosePub->resetPoseInfo();
        return ret;
    }
    void resetAdjustParam()
    {
        mPreTime=-1;
        mPreIMU=Eigen::Vector3d(0.0,0.0,0.0);
        mIMUcnt=0;
        mAngleVelSum=0;
        mAngleError=0;
        mAngleVelError=0;
        mPreAngleVel=0;
        mlpWZ = 0;
    }
   void setupMotor()
    {
        mMotor->powerOFF();
        start_motor_daemon();
    }
    void loadCalibra()
    {
        int i;
        double gyro[3];
        double accel[3];

        readVector3fCalibration(GYRO_CALIBRATION_FILE,gyro);
        readVector3fCalibration(ACC_CALIBRATION_FILE,accel);
        mCalibra.x()=accel[0];
        mCalibra.y()=accel[1];
        mCalibra.z()=gyro[2];
        ROS_DEBUG("loadCalibra:%f %f %f",mCalibra.x(), mCalibra.y(), mCalibra.z());
    }
    void fixMotorDir(float& vx,float& vy)
    {
        if(!mDirFixMode){
            return;
        }
        ROS_DEBUG("frame angle:%f",mDirAngle);
        float x,y;
        float c=cos(mDirAngle),s=sin(mDirAngle);
        x=vx*c+vy*s;
        y=vy*c-vx*s;
        vx=x;
        vy=y;
    }
    void autoCalibraIMU(double x,double y,double w)
    {
        if (mBPatrolCalib && mIMUCalibraCnt<IMU_CALIBRA_CNT){
            mIMUCalibraOffset+=Eigen::Vector3d(x,y,w);
            if(++mIMUCalibraCnt==IMU_CALIBRA_CNT){
                mCalibra+=mIMUCalibraOffset/IMU_CALIBRA_CNT;
                mBPatrolCalib = false;
            }
        }else if(mIMUCalibraCnt<IMU_CALIBRA_CNT){
            if(abs(w)<GYRO_CALIBRA_THRESHOLD &&
                   abs(x)<ACCEL_CALIBRA_THRESHOLD&&
                   abs(y)<ACCEL_CALIBRA_THRESHOLD){
                mIMUCalibraOffset+=Eigen::Vector3d(x,y,w);
                if(++mIMUCalibraCnt==IMU_CALIBRA_CNT){
                    mCalibra+=mIMUCalibraOffset/IMU_CALIBRA_CNT;
                }
            }else{
                resetAutoIMUCalibra();
            }
        }
    }

    void resetAutoIMUCalibra()
    {
        mIMUCalibraCnt=0;
        mIMUCalibraOffset=Eigen::Vector3d(0.0,0.0,0.0);
    }
    void batteryStatusCallback(const statusConstPtr &s)
    {
        mChargingStatus=s->status[2];
    }
    void navStatusCallback(const statusConstPtr &status)
    {
        if(status->status[0]){
            if(!mTof){
                ROS_DEBUG("open tof");
                mTof=mHandle.subscribe("SensorNode/tof",1,&MotorDriver::tofCallback,this);
            }
        }else{
            if(mTof){
                ROS_DEBUG("close tof");
                mAvgTof = MAX_VALID_TOF_DIST;
                mTof.shutdown();
            }
            mDistance=DBL_MAX;
        }
    }

    void goingHomeStatus(const std_msgs::Int32::ConstPtr& msg)
    {
        switch (msg->data)
        {
        case status::BACK_UP_SUCCESS:
        case status::BACK_UP_FAIL:
        case status::BACK_UP_INACTIVE:
         if(mTof){
                ROS_DEBUG("goingHomeStatus close tof");
                mAvgTof = MAX_VALID_TOF_DIST;
                mTof.shutdown();
                resetAdjustParam();
            }
            mDistance=DBL_MAX;
            break;
        case  status::BACK_UP_DETECT:
           if(!mTof){
                ROS_DEBUG("open tof");
                mTof=mHandle.subscribe("SensorNode/tof",1,&MotorDriver::tofCallback,this);
            }
        break;
        default:
            break;
        }
    }

    void tofCallback(const sensor_msgs::RangeConstPtr &r)
    {
        float range = r->range;
        if (range > MAX_VALID_TOF_DIST || range<0){
            range = MAX_VALID_TOF_DIST;
        }
        mAvgTof = mAvgTof+(range-mAvgTof)/mTofAvgCnt;
        mDistance=r->range;
    }

    bool imuPatrolCalib(imu_patrol_calibRequest& req,imu_patrol_calibResponse& res)
    {
#ifndef SENSOR_IMU_USE_CALIBRA
        loadCalibra();
#endif
        mBPatrolCalib = true;
        resetAutoIMUCalibra();
        return true;
    }

    bool imuPatrolCalibStatus(getimu_patrolcalib_statusRequest& req,getimu_patrolcalib_statusResponse& res)
    {
        res.ret = mIMUCalibraCnt < IMU_CALIBRA_CNT;
        return true;
    }

    DeviceDefaultConfig mCfg;
    ros::NodeHandle mHandle;
    ros::Subscriber mCmdVel;
    ros::Subscriber mForceCmdVel;
    ros::Subscriber mCmdVel2;
    ros::Subscriber mCmdVel3;
    ros::Subscriber mCmdVel4;
    ros::Subscriber mCmdVel8003;
    ros::Subscriber mIMU;

    mutex mMutex;
    condition_variable mCondi;
    thread mMotorDaemon;

    shared_ptr<Motor> mMotor;
    bool mRunning;
    bool mPowerOn;
    bool mStopped;
    bool mKeepMove;
    bool mMotorEnable;
    bool mEnableDynamicAdjust;
    //bool mBImuPatrolCalib;
    int STOP_TIME_THRESHOLD;
    int MOTOR_POWEROFF_TIMEOUT;
    int mMotorPowerOffTimeout;

    double mPreTime;
    Eigen::Vector3d mPreIMU;
    double mAngleError;
    double mlpWZ;
    int mIMUcnt;
    double mAngleVelSum;
    double mAngleVelError;
    double mPreAngleVel;
    int mSkipAjustCnt;
    Eigen::Vector3d mCalibra;

    bool mDirFixMode;
    float mDirAngle;

    shared_ptr<MotorPosePublisher> mPosePub;

    double  GYRO_CALIBRA_THRESHOLD;
    double  ACCEL_CALIBRA_THRESHOLD;
    int  IMU_CALIBRA_CNT;
    int mIMUCalibraCnt;
    bool mBPatrolCalib;
    Eigen::Vector3d mIMUCalibraOffset;

    ros::Subscriber mBattery;
    int mChargingStatus;
    uint64_t mCmdVelCnt;

    ros::Timer mTimer;
    ros::Subscriber mNav;
    ros::Subscriber mTof;
    ros::Subscriber mAlgBack;
    ros::ServiceServer mImuCalib;
    ros::ServiceServer mImuPatrolCalibStatus;
    float OBSTACLE_MIN_DISTANCE;
    float OBSTACLE_SPEED;
    float SLOW_SPEED_DISTANCE;
    double mDistance;
    int mMinDisCnt;
    int mDoubleMinDistCnt;
    int mTofAvgCnt;
    float mAvgTof;
    float mVx;
    float mVy;
    float mVw;
};

static shared_ptr<MotorDriver> g_motor_driver;
class DisableMotor{
public:
    DisableMotor()
    {
    }
    ~DisableMotor()
    {
    }
    void start()
    {
        g_motor_driver->enable(false);
    }
    void stop()
    {
        g_motor_driver->enable(true);
    }

    int setData(std_msgs::String& s)
    {
        s.data="motor hello";
        return 0;
    }
};
class DisableMotorDynamicAdjust{
public:
    DisableMotorDynamicAdjust()
    {
    }
    ~DisableMotorDynamicAdjust()
    {
    }
    void start()
    {
        g_motor_driver->enable_adjust(false);
    }
    void stop()
    {
        g_motor_driver->enable_adjust(true);
    }

    int setData(std_msgs::String& s)
    {
        s.data="dynamic adjust hello";
        return 0;
    }
};
class FixMotorMoveDirection{
public:
    FixMotorMoveDirection()
    {
    }
    ~FixMotorMoveDirection()
    {
    }
    void start()
    {
        g_motor_driver->enable_dir_fixed(true);
    }
    void stop()
    {
        g_motor_driver->enable_dir_fixed(false);
    }
    int setData(std_msgs::String& s)
    {
        s.data="motor direction hello";
        return 0;
    }
};

int main(int argc, char **argv)
{
  InitDZLog dzLog;
  ros::init(argc, argv, "MotorNode");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,MOTOR_NODE_DEBUG_LEVEL);
  g_motor_driver=make_shared<MotorDriver>();
  auto g=make_shared<ros::NodeHandle>("~");
  DataPulisher<std_msgs::String,DisableMotor> disableMotor(g,"disable",2,make_shared<DisableMotor>(),1.0);
  DataPulisher<std_msgs::String,DisableMotorDynamicAdjust> disableMotorAdjust(g,"disable_adjust",2,make_shared<DisableMotorDynamicAdjust>(),1.0);
  DataPulisher<std_msgs::String,FixMotorMoveDirection> enalbeMotorFixDir(g,"enable_fix_dir",2,make_shared<FixMotorMoveDirection>(),1.0);
  DataPulisher<nav_msgs::Odometry,MotorPosePublisher> motorPosePub(g,"baselink_odom_relative",100,g_motor_driver->getPosePublisher(),10.0);
  DataPulisher<nav_msgs::Odometry,VioPosePublisher> vioPosePub(g,"vio_odom_relative",100,make_shared<VioPosePublisher>(),10.0);
  ros::spin();
  return 0;
}