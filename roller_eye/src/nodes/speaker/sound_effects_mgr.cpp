#include <ros/ros.h>
#include"roller_eye/system_event.h"

#include "roller_eye/sound_effects_mgr.h"
#include"roller_eye/nav_get_status.h"
#include "roller_eye/util_class.h"
#include "roller_eye/plt_config.h"
#include "roller_eye/patrol_status.h"
#include"nlohmann/json.hpp"
#include "roller_eye/speaker_node.h"

using json = nlohmann::json;


typedef enum PATROL_STATE_T{
    PATROL_NONE = 0,
    PATROL_RUNNING,
    PATROL_PILING,
    PATROL_PILE_FAILUER
}PATROL_STATE;

std::string s_sound_files[]={
    "/var/roller_eye/devAudio/power_on_down/power_on.wav",
    "/var/roller_eye/devAudio/power_on_down/power_down.wav",
    "/var/roller_eye/devAudio/connect/Iot2WifiDirect.wav",
    "/var/roller_eye/devAudio/connect/WifiDirect2Iot.wav",
    "/var/roller_eye/devAudio/connect/connect_wifi.wav",
    "/var/roller_eye/devAudio/connect/success.wav",
    "/var/roller_eye/devAudio/connect/fail.wav",
    "/var/roller_eye/devAudio/go_leave_home/charging.wav",
    "/var/roller_eye/devAudio/go_leave_home/leave_home.wav",
    "/var/roller_eye/devAudio/battery_low/battery_low.wav",
    "/var/roller_eye/devAudio/got_new_instruction/got_new_instrction.wav",
    "/var/roller_eye/devAudio/detected/obstacle_detected.wav",
    "/var/roller_eye/devAudio/rest_to_factory_condition.wav",
    "/var/roller_eye/devAudio/alert/alert2.wav",
    "/var/roller_eye/devAudio/searching/searching 02.wav"
    "/var/roller_eye/devAudio/detected/obstacle_detected.wav"
};

namespace roller_eye
{
    SoundEffectsMgr::SoundEffectsMgr(shared_ptr<ros::NodeHandle> &node):
    mNodeHandle(node),
    mPrevPercentage(100),
    mCurrentPercentage(0),
    mRunning(false),
    mChargingStatus(status::BATTERY_UNKOWN),
    mPrevChargingStatus(status::BATTERY_UNKOWN)
    {
        mPatrolState = PATROL_NONE;
        mPlayingFlagVec.resize(SYSEVT_SND_MAX, false);
        struct timespec timestamp = { 0, 0 };
        clock_gettime(CLOCK_MONOTONIC, &timestamp);
        /*if(timestamp.tv_sec<5*60){  //5分钟内都认为刚开机
            PlayAudio(SYSEVT_POWERON);
        } */
        mGoingHomeStatusSub = mNodeHandle->subscribe("CoreNode/going_home_status", 1, &SoundEffectsMgr::goingHomeStatus,this); //找充电桩状态
        mGetNavStatusClient      = mNodeHandle->serviceClient<nav_get_status>("NavPathNode/nav_get_status");
        mSysEventSvr                     = mNodeHandle->advertiseService("system_event",&SoundEffectsMgr::SystemEvent,this);
        mSysEvtPub                        = mNodeHandle->advertise<std_msgs::Int32>("/system_event",1);
        mPatrolStatusPub = mNodeHandle->advertise<patrol_status>("/patrol_status",1);

        mMinBat = LOW_BATTERY_PER;
        PltConfig::getInstance()->getMinBat(mMinBat);

        mPatrolStatsuSub = mNodeHandle->subscribe("/patrol_status", 10,
                                                                                                &SoundEffectsMgr::patrolStatus, this);

        if (!mPlayAudioSub){
            mPlayAudioSub = mNodeHandle->subscribe("/system_event", 10,
                                                                                                &SoundEffectsMgr::PlayAudioCallback, this);
        }
        if (!mWiFiSub){
            mWiFiSub  = mNodeHandle->subscribe("WiFiNode/status", 10,
                                                                                                &SoundEffectsMgr::wifiStatusCallback,this);
        }
        if (!mBatteryStatus){
            mBatteryStatus  = mNodeHandle->subscribe("SensorNode/simple_battery_status",
                                                                                                            10, &SoundEffectsMgr::batteryStatusCallback,this);
        }
        Start();
    }

    SoundEffectsMgr::~SoundEffectsMgr()
    {
        if (mWiFiSub){
            mWiFiSub.shutdown();
        }
        if (mPlayAudioSub){
            mPlayAudioSub.shutdown();
        }
        if (mBatteryStatus){
            mBatteryStatus.shutdown();
        }

    }

    void SoundEffectsMgr::Start()
    {
        if (!mRunning){
            mPlayThread = std::thread(&SoundEffectsMgr::Loop, this);
        }
    }

    void SoundEffectsMgr::patrolStatus(const patrol_status& msg)
    {
        switch(msg.type){
            case patrol_status::START_PATROL:
            UpdatePatrolState(PATROL_RUNNING);
            break;
            case patrol_status::END_PATROL:
            UpdatePatrolState(PATROL_PILING);
            break;
            default:
            break;
        }
    }

    void SoundEffectsMgr::PlayAudioCallback(const std_msgs::Int32::ConstPtr& msg)
    {
        SYSEVT_T evt = static_cast<SYSEVT_T>(msg->data);

        if(SYSEVT_MOTOR_BYCMD == evt){
            mPatrolState = PATROL_NONE;
            return;
        }

        if (!mRunning ||  evt>=SYSEVT_SND_MAX || evt<=SYSEVT_NONE){
            ROS_INFO("SoundEffectsMgr illegal id %d",evt);
            return;
        }
         PlayAudio(evt);
    }

    void SoundEffectsMgr::Stop()
    {
        mRunning = false;

        PLOG_INFO(SPEAKER_NODE_TAG,"Stop SoundEffectsMgr");
        if(detectProcessIsExited("aplay")){
	        system("sudo killall aplay");
        }
        if (mPlayThread.joinable()){
            mPlayThread.join();
        }
        mPlayingFlagVec.resize(SYSEVT_SND_MAX, false);
        queue<SYSEVT_T> empty;
        std::swap(empty, mSoundQue);
    }

    void SoundEffectsMgr::Loop()
    {
        PLOG_INFO(SPEAKER_NODE_TAG,"start SoundEffectsMgr");
        mRunning = true;
        while(mRunning){
            SYSEVT_T evt = GetSoundID();
            if (evt>SYSEVT_NONE && evt<SYSEVT_SND_MAX){
                Play(evt);
            }else{
                usleep(20*1000);
            }
        }
    }

    SYSEVT_T SoundEffectsMgr::GetSoundID()
    {
            SYSEVT_T evt =  SYSEVT_NONE;
            mMutex.lock();
            if (!mSoundQue.empty()){
                evt = mSoundQue.front();
                mSoundQue.pop();

                mPlayingFlagVec[evt] = false;
            }
            mMutex.unlock();

            return evt;
    }

    int SoundEffectsMgr::PlayAudio(SYSEVT_T evt)
    {
        int ret = -1;
        if (!mRunning || SYSEVT_NONE == evt){
            return ret;
        }

        mMutex.lock();
        if (!mPlayingFlagVec[evt]){
            ret = 0;
            mSoundQue.push(evt);
            mPlayingFlagVec[evt] = true;
        }else{
            ROS_INFO("SoundEffectsMgr %d waiting play",evt);
        }
        mMutex.unlock();
         return ret;
    }

    void SoundEffectsMgr::Play(SYSEVT_T evt)
    {
        if (!mRunning){
            return;
        }

        if (evt >=SYSEVT_SND_MAX){
            return;
        }

        json param;
        PltConfig *config = PltConfig::getInstance();
        config->getSoundEffectParam(param);
        int activate = param["activate"];
        if (!activate){
            return;
        }
        if (SYSEVT_OBJDETECTED == evt){
            if (param["detectEffect"].is_null()){
                return;
            }

            int detectEffect = param["detectEffect"];
            if (!detectEffect){
                return;
            }
        }

        string cmd = "aplay "+s_sound_files[evt];
        system(cmd.c_str());
    }

    bool SoundEffectsMgr::SystemEvent(system_eventRequest& req, system_eventResponse& res)
    {
        if(SYSEVT_MOTOR_BYCMD == req.event){
            mPatrolState = PATROL_NONE;
        }

        if (!mRunning ||
                req.event<=SYSEVT_NONE ||
                req.event>=SYSEVT_SND_MAX){
                return false;
        }

        return SoundEvent(req, res);
    }


    bool SoundEffectsMgr::SoundEvent(system_eventRequest& req, system_eventResponse& res)
    {
        if (SYSEVT_POWEROFF == req.event){
            Stop();
            Play(static_cast<SYSEVT_T>(req.event));
        }else{
            PlayAudio(static_cast<SYSEVT_T>(req.event));
        }
        return true;
    }

   void SoundEffectsMgr::wifiStatusCallback(const statusConstPtr &s)
    {
        if (!mRunning){
            return;
        }

        int mode =s->status[0];
        int status =s->status[1];
        switch (status)
        {
        case status::WIFI_STATUS_STOP:
            PlayAudio(SYSEVT_WIFIDISCON);
            break;
        case status::WIFI_STATUS_CONNECTING:
            PlayAudio(SYSEVT_SETWIF);
            break;
        case status::WIFI_STATUS_CONNECTED:
            PlayAudio(SYSEVT_WIFICON);
            break;
        case status::WIFI_STATUS_WRONG_KEY:
            PlayAudio(SYSEVT_WIFIDISCON);
             break;
        case status::WIFI_STATUS_DISCONNECT:
            PlayAudio(SYSEVT_WIFIDISCON);
             break;
        case status::WIFI_STATUS_CONN_FAIL:
            PlayAudio(SYSEVT_WIFIDISCON);
            break;
        default:
            break;
        }
    }

    void SoundEffectsMgr::batteryStatusCallback(const statusConstPtr &s)
    {
        if (!mRunning){
            return;
        }

        static int nCnt = 0;
        mCurrentPercentage=s->status[1];
        mChargingStatus=s->status[2];
        if(!mChargingStatus &&
               mPrevPercentage>mMinBat && mCurrentPercentage<=mMinBat){
               PlayAudio(SYSEVT_LOWPOWER);
        }
        if (mChargingStatus){
            nCnt = 0;
            mPrevPercentage = 100;
            if (mPrevChargingStatus != mChargingStatus){
                mPatrolState = PATROL_NONE;
                PlayAudio(SYSEVT_STARTCHARGING);
            }
        }else{
            mPrevPercentage = mCurrentPercentage;
        }

        mPrevChargingStatus = mChargingStatus;
    }

     void SoundEffectsMgr::goingHomeStatus(const std_msgs::Int32::ConstPtr& msg)
     {
        switch (msg->data)
        {
        case status::BACK_UP_DETECT:
        case status::BACK_UP_ALIGN:
        case status::BACK_UP_BACK:
            UpdatePatrolState(PATROL_PILING);
            break;
        case status::BACK_UP_SUCCESS:
        case status::BACK_UP_CANCEL:
                UpdatePatrolState(PATROL_NONE);
            break;
        case status::BACK_UP_FAIL:
                UpdatePatrolState(PATROL_PILE_FAILUER);
        break;
        default:
            break;
        }
     }

    void SoundEffectsMgr::UpdatePatrolState(int state)
    {
        switch(state){
            case PATROL_NONE: {
                    mPatrolState = PATROL_NONE;
            }
            break;
            case PATROL_RUNNING:{
                if (PATROL_NONE == mPatrolState){
                    PltConfig *config = PltConfig::getInstance();
                    mPatrolState = PATROL_RUNNING;
                    auto th=std::thread([this, config](){
                        while(PATROL_RUNNING == mPatrolState){
                            json param;
                            config->getSoundEffectParam(param);
                            int activate = param["activate"];
                            int patrolEffect = param["patrolEffect"];
                            if (activate && patrolEffect){
                                string cmd = "aplay /var/roller_eye/devAudio/alert/alert.wav";
                                system(cmd.c_str());
                            }
                            usleep(100*1000);
                        }
                    });
                    th.detach();
                }
            }
            break;
            case PATROL_PILING:
                mPatrolState = PATROL_PILING;
            break;
            case PATROL_PILE_FAILUER:{
                if (PATROL_PILING == mPatrolState){
                        mPatrolState = PATROL_PILE_FAILUER;
                        auto th=std::thread([this](){
                            while(PATROL_PILE_FAILUER == mPatrolState){
                                json param;
                                PltConfig *config = PltConfig::getInstance();
                                config->getSoundEffectParam(param);
                                int activate = param["activate"];
                                int patrolEffect = param["patrolEffect"];
                                if (activate){
                                    string cmd = "aplay /var/roller_eye/devAudio/go_leave_home/charging.wav";
                                    system(cmd.c_str());
                                    usleep(100*1000);
                                }
                            }
                        });
                    th.detach();
                }
                patrol_status status;
                status.type = patrol_status::PATROL_LOSE_PILE;
                mPatrolStatusPub.publish(status);

            }
            break;
            default:
            break;
        }

    }
}