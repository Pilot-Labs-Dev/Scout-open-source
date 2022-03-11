#include<unistd.h>
#include<cmath>
#include <thread>
#include"roller_eye/algo_utils.h"
#include"geometry_msgs/Twist.h"
#include"roller_eye/system_define.h"
#include"roller_eye/util_class.h"
#include "roller_eye/stop_detect.h"
#include "roller_eye/system_define.h"
#include"roller_eye/plt_config.h"

#define G_NORMAL     9.7833
#define WAITING_OBJ_TIME       50

#define MIN_ROLL_SPEED              1.0f
#define FS8003_MIN_ROLL_SPEED 1.4f
#define IMU_WATING_TIME            100
#define ROLL_DURATION                 25
#define ROLL_IMU_CALIBRA_CNT  20  

#define CONNECT_CHARGE_PILE_ONCE_TIME   250

//#define GEN_DISTANCE_SPEED          1.5
#define GEN_DISTANCE_SPEED          1.1

#define MAG_CALIBRA_PARAM_CNT       5
#define MAG_CALIBRA_GAIN                        (1.0e5)

using namespace std;

namespace roller_eye
{
    AlgoUtils::AlgoUtils():
    mHandle(""),
    mHasDetected(false),
    mQuitFlag(false),
    mAvgTof(MAX_VALID_TOF_DIST),
    mTofAvgCnt(ALGO_TOF_AVG_COUNT)
    {
        mMotorVel=mHandle.advertise<geometry_msgs::Twist>("cmd_vel",50);
        mMotorVel2 =mHandle.advertise<geometry_msgs::Twist>("cmd_vel2",50);

        mStopDetectClient = mHandle.serviceClient<stop_detect>("CoreNode/stop_detect");
        mMinRollSpeed = MIN_ROLL_SPEED;
        bool bFs8003Motor = false;
        string hwVer;
        PltConfig::getInstance()->getHwVersion(hwVer);
        if (hwVer.length() ==10 && hwVer[5] == '0' && hwVer[6] == '1' ){
            mMinRollSpeed = FS8003_MIN_ROLL_SPEED;
        }
        //mTof=mHandle.subscribe("SensorNode/tof",1,&AlgoUtils::onTofData,this);
    }
    AlgoUtils::~AlgoUtils()
    {

    }

    void AlgoUtils::waitObjBegin()
    {
        if(!mObjDetect){
            mObjDetect=mHandle.subscribe("CoreNode/chargingPile",10,&AlgoUtils::onObjdetect,this);              
        }
    }
    void AlgoUtils::waitObjEnd()
    {
        shutdownObjDetect();
    }

    int AlgoUtils::waitObj(const string& name,int timeout,AlgoOBjPos &pos)
    {
        bool objDetectOpen=false;
        
        struct timeval tm;
        gettimeofday(&tm, nullptr);
        usleep(100*1000);    
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
                    ROS_INFO("waitObj get obj:%ld,obj->stamp:%ld",mStartWaitObjStamp, pos.stamp);
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
        ROS_INFO("waitObj get obj failured!\n");
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
                    ROS_INFO("waitObj get obj:%ld,obj->stamp:%ld",mStartWaitObjStamp, pos.stamp);
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

        return ret;
    }

    void AlgoUtils::quitAlgo()
    { 
        mQuitFlag= true;
        shutdownObjDetect();
    }


    void AlgoUtils::stopDetectPile()
    { 
        auto th=std::thread([this](){
            stop_detect req;
            req.request.cmd = 0;
            mStopDetectClient.call(req);      
        });
        th.detach();
    }

    void AlgoUtils::shutdownObjDetect()
    {
        if (mObjDetect){
            mObjDetect.shutdown();
        }

        stopDetectPile();
       
    }

    void AlgoUtils::moveOnce(float vx,float vy,float wz,int duration,int stop)
    {     
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

    void AlgoUtils::moveOnceEx(float vx,float vy,float wz,float dist, int duration,int stop)
    {
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
        DistanceCloud dist;
        return rollAndGenDistance(angle,w,timeout,error,false,dist);
    }

    int AlgoUtils::rollEx(float angle,float w,int timeout,float error)
    {
        DistanceCloud dist;
        return rollAndGenDistanceEx(angle,w,timeout,error,false,dist);
    }

    int AlgoUtils::rollAndGenDistance(float angle,float w,int timeout,float error,bool genDist,DistanceCloud& dist)
    {
        int time,ret=0,dur;
        float rolled,err,wz;

        //ROS_INFO("rollAndGenDistance mQuitFlag: %d angle:%f  w:%f %s %d \n", mQuitFlag, angle, w, __FILE__, __LINE__);          
        if(abs(angle)<=0.025){
          return 0;
        }

        moveOnce(0.0,0.0,0.0,1000,0);
        if(w<mMinRollSpeed){
            w=mMinRollSpeed;
        }

        mRollAngular=0.0;
        mIsFirstIMU=true;
        mIMUCalibraCnt=0;
        mZCalibra=0.0;
        mIMU=mHandle.subscribe("SensorNode/imu",1000,&AlgoUtils::onIMUData,this);
        if(genDist){
            mDistance=FLT_MAX;
            mTof=mHandle.subscribe("SensorNode/tof",1,&AlgoUtils::onTofData,this);
        }
        time=0;

        int tries = 0;
        int maxTries = timeout/ROLL_DURATION;
     
        while(1){
            if(mQuitFlag ||               
                (timeout>=0 && time>timeout)){
                ret=-1;
                break;
            }		    
            if(mIsFirstIMU){
                usleep(IMU_WATING_TIME*1000);
                time+=IMU_WATING_TIME; 
                continue;
            }
            mAngleMutex.lock();
            rolled=mRollAngular;
            if(genDist){
                dist.push_back(rolled);
                dist.push_back(mDistance);     
            }
            mAngleMutex.unlock();
            err=abs(rolled-angle);
            if(err<error){
                break;
            }
 
            wz=err/abs(angle)*(w-mMinRollSpeed)+mMinRollSpeed;
            if (abs(wz)>abs(w)){
                wz = w;
            }
            dur=(int)(err*1000/wz);
            if(dur>ROLL_DURATION){
                dur=ROLL_DURATION;
            }
   
            wz*=rolled>angle?-1:1;

            moveOnce(0.0,0.0,wz,dur,0);

            time+=dur;
        }
        moveOnce(0.0,0.0,0.0,0,50);
        mIMU.shutdown();
       if(genDist){
            mTof.shutdown();
       }
       resetQuitFlag();
        // api_set_thread_policy (&attr, policy);
        // pthread_attr_destroy (&attr);
        return ret;
    }

    int AlgoUtils::rollAndGenDistanceRm(float angle,float w,int timeout,float error,bool genDist,shared_ptr<DistanceMap> distMap)
    {
        int time,ret=0,dur;
        float rolled,err,wz;
      
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
        if(w<mMinRollSpeed){
            w=mMinRollSpeed;
        } 
        
        mRollAngular=0.0;
        mIsFirstIMU=true;
        mIMUCalibraCnt=0;
        mZCalibra=0.0;
        mIMU=mHandle.subscribe("SensorNode/imu",1000,&AlgoUtils::onIMUData,this);


        time=0;
		static int genDistIndex = 0;

		genDistIndex = 0;
        while(1){
            if(mQuitFlag || 
                 (timeout>=0 && time>timeout)){
                ret=-1;
                break;
            }
            if(mIsFirstIMU){
                usleep(IMU_WATING_TIME*1000);
                time+=IMU_WATING_TIME;
                continue;
            }
            mAngleMutex.lock();
            rolled=mRollAngular;
            if(genDist){
            	if(FLT_MAX != mDistance){
					pair<double, double> DistPoint(rolled, mDistance);
					distMap->push_back(DistPoint);
					genDistIndex++;
            	}
            }
            mAngleMutex.unlock();
            err=abs(rolled-angle);
            if(err<error){
                break;
            }
            wz=err/abs(angle)*(w-MIN_ROLL_SPEED)+MIN_ROLL_SPEED;
            if (abs(wz)>abs(w)){
                wz = w;
            }
            dur=(int)(err*1000/wz);
            if(dur>ROLL_DURATION){
                dur=ROLL_DURATION;
            }

            wz*=rolled>angle?-1:1;

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
        int time,ret=0,dur;
        float rolled,err,wz;

        if(abs(angle)<=0.025){
          return 0;
        }

        moveOnce(0.0,0.0,0.0,1000,0);
        if(w<mMinRollSpeed){
            w=mMinRollSpeed;
        }

        mRollAngular=0.0;
        mIsFirstIMU=true;
        mIMUCalibraCnt=0;
        mZCalibra=0.0;
        mIMU=mHandle.subscribe("SensorNode/imu",1000,&AlgoUtils::onIMUData,this);
        if(genDist){
            mDistance=FLT_MAX;
            mTof=mHandle.subscribe("SensorNode/tof",1,&AlgoUtils::onTofData,this);
        }
        time=0;


        int tries = 0;
        int maxTries = timeout/ROLL_DURATION;
        while(1){
            ros::spinOnce();
            if(mQuitFlag ||      
                (timeout>=0 && time>timeout)){
                ret=-1;
                break;
            }
            if(mIsFirstIMU){
                usleep(IMU_WATING_TIME*1000);
                time+=IMU_WATING_TIME;
                continue;
            }
            mAngleMutex.lock();
            rolled=mRollAngular;
            if(genDist){
                dist.push_back(rolled);
                dist.push_back(mDistance);
            }
            mAngleMutex.unlock();
            err=abs(rolled-angle);
            if(err<error){
                break;
            }
 
            wz=err/abs(angle)*(w-mMinRollSpeed)+mMinRollSpeed;
            if (abs(wz)>abs(w)){
                wz = w;
            }
            dur=(int)(err*1000/wz);
            if(dur>ROLL_DURATION){
                dur=ROLL_DURATION;
            }
 
            wz*=rolled>angle?-1:1;

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

    int AlgoUtils::genDistance(DistanceCloud& dist)
    {
        dist.resize(0);
        return rollAndGenDistance(M_PI*2,GEN_DISTANCE_SPEED,DURATION_10_S,DGREE_3_RAD,true,dist);
    }
    
    int AlgoUtils::getDistance(float angle,shared_ptr<DistanceMap> distMap)
    {
        distMap->clear();
        return rollAndGenDistanceRm(angle,GEN_DISTANCE_SPEED,DURATION_10_S,DGREE_3_RAD,true,distMap);
    }
    
    float AlgoUtils::getMaxSpace(DistanceCloud& dist,float distThreshold,float& arc)
    {
        float maxAngle=-1.0;
        float maxIdx=-1.0;
        float startIdx=-1.0;
        float angle;

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
        if(s<1e-3){
            return 0;
        }
        float speedX=speed*x/s;
        float speedY=speed*y/s;
        int duration=std::max(int(s/speed*1000/speed_e),250);
        moveOnce(speedX,speedY,0.0,duration,1);
        return 0;
    }

    int AlgoUtils::moveEx(float x,float y,float speed,int timeout,float error)
    {
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
        moveOnce(speedX,speedY,speedW,time,1);
        return 0;
    }

    void AlgoUtils::onBatteryStatus(const statusConstPtr &s)
    {
        mCharging=s->status[2];
    }

    /**
     * @description: when needed to be called-?
     * @param {*}
     * @return {*}
     */
    int AlgoUtils::connectToChargePile(int timeout)
    {
        bool bFs8003Motor = false;
        string hwVer;
        PltConfig::getInstance()->getHwVersion(hwVer);
        if (hwVer.length() ==10 && hwVer[5] == '0' && hwVer[6] == '1' ){
            bFs8003Motor = true;
        }

        int t=0;
        mCharging=-1;
        mBatteryStatus=mHandle.subscribe("SensorNode/simple_battery_status",10,&AlgoUtils::onBatteryStatus,this);


        const int _ms100=100*1000;
        while(mCharging<0 ){
            usleep(_ms100);
            t+=_ms100;
            if(t>10 * _ms100){
                break;
            }
            if( mQuitFlag){    
                return -1;
            }
        }
        t=0;
        float vy = -0.14;
        if (bFs8003Motor){
            timeout *= 2;
            vy = -0.2;
        }
        int sleepTm = 200;
        while(!mQuitFlag){
            moveOnce(0,vy,0,CONNECT_CHARGE_PILE_ONCE_TIME,1000);   

            for (int i=0; i<10; i++){
                usleep(sleepTm*1000);
                if (mCharging > 0 ){
                    break;
                }             
                if (mQuitFlag){
                    break;
                }  
            }
            if (mCharging > 0){
                break;
            }
            if(t>timeout){
                break;
            }
            t+=CONNECT_CHARGE_PILE_ONCE_TIME;
        }

        usleep(1000*1000);
        mBatteryStatus.shutdown();
        resetQuitFlag();
        if (mCharging<=0){
            return -1;
        }
        return 0;
    }

    bool AlgoUtils::isCharging()
    {
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
            }       
        }        
    }
    void AlgoUtils::onIMUData(const sensor_msgs::ImuConstPtr& imu)
    {
        if(mIMUCalibraCnt<ROLL_IMU_CALIBRA_CNT){
            mZCalibra+=imu->angular_velocity.z;
            if(++mIMUCalibraCnt==ROLL_IMU_CALIBRA_CNT){
                mZCalibra/=ROLL_IMU_CALIBRA_CNT;
            }
            return;
        }
        if(mIsFirstIMU){
            mIsFirstIMU=false;
            mPreIMU=*imu;
            return;
        }
        auto t=(imu->header.stamp-mPreIMU.header.stamp).toSec();
        mAngleMutex.lock();
        mRollAngular+=((mPreIMU.angular_velocity.z+imu->angular_velocity.z)/2-mZCalibra)*t;
        mAngleMutex.unlock();
        mPreIMU=*imu;
    }

    void AlgoUtils::onTofData(const sensor_msgs::RangeConstPtr &r)
    {
        mAngleMutex.lock();		
		float range = r->range;
		if (isinf(r->range) || r->range <0){
			range = MAX_VALID_TOF_DIST;
		}
		mAvgTof = mAvgTof+(range-mAvgTof)/mTofAvgCnt;
		mDistance=mAvgTof;
		mIsFirstTof = false;
        mAngleMutex.unlock();
    }
     void AlgoUtils::magneticCalibrate(int timeout)
     {
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

        saveCalibration(MAG_CALIBRATION_FILE,calibra,MAG_CALIBRA_PARAM_CNT);
     }
    void AlgoUtils::onMagData(const sensor_msgs::MagneticFieldConstPtr &mag)
    {
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
        Eigen::Vector3d t(msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);
        float vx=msg->twist.twist.linear.x;
        float vy=msg->twist.twist.linear.y;
        float w=msg->twist.twist.angular.z;

        mCurPostion+=mCurPose*t;
        mCurPose=mCurPose*Eigen::Quaterniond(msg->pose.pose.orientation.w,msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z);
   
    }

    void loadMagCalibraParam(MagCalibraParam& param)
    {
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
        //ROS_INFO("mag calibra:%f,%f,%f,%f,%f,%f",param.x,param.y,param.w,param.h,param.c,param.s);
    }
    void magCalibrate(double& x,double&y,double&z,MagCalibraParam& param)
    {
        double x1=x-param.x;
        double y1=y-param.y;
        double avg=(param.w+param.h)/2;

        x=(x1*param.c-y1*param.s)/param.w*avg;
        y=(x1*param.s+y1*param.c)/param.h*avg;
    }
} // namespace roller_eye
