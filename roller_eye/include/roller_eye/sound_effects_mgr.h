#pragma once

#include <thread>
#include <queue>
#include <vector>
#include"ros/ros.h"
#include"roller_eye/status.h"
#include "roller_eye/system_event.h"
#include "roller_eye/patrol_status.h"
#include "std_msgs/Int32.h"
#include"roller_eye/single_class.h"


typedef enum SYS_EVENT_TYPE{
    SYSEVT_NONE = -1,
    SYSEVT_POWERON,                         
    SYSEVT_POWEROFF,                        
    SYSEVT_STA2AP,                                 
    SYSEVT_AP2STA,                                
    SYSEVT_SETWIF,                                
    SYSEVT_WIFICON,                             
    SYSEVT_WIFIDISCON,                     
    SYSEVT_STARTCHARGING,            
    SYSEVT_OUTPILE,                            
    SYSEVT_LOWPOWER,                    
    SYSEVT_NETCMD,                          
    SYSEVT_OBSTACLE,                        
    SYSEVT_REST_TO_FACTORY,       
    SYSEVT_OBJDETECTED,                 
    SYSEVT_WIFICONNECTING,          
    SYSEVT_TRACEDONE,                     
    SYSEVT_SND_MAX,
    SYSEVT_MOTOR_BYCMD,           
    SYSEVT_CANT_SAVE,                     
    SYSEVT_BIST_START,              
}SYSEVT_T;

namespace roller_eye
{
class SoundEffectsMgr{ 
    public:
    SoundEffectsMgr(shared_ptr<ros::NodeHandle> &node);
    ~SoundEffectsMgr();

    int    PlayAudio(SYSEVT_T evt);
    void Start();
    void Stop();

private:
    void Loop();
    SYSEVT_T GetSoundID();
    void Play(SYSEVT_T evt); 
    void PlayAudioCallback(const std_msgs::Int32::ConstPtr& msg);    
    void wifiStatusCallback(const statusConstPtr &s);
    void batteryStatusCallback(const statusConstPtr &s);
    void patrolStatus(const patrol_status& msg);

    bool SystemEvent(system_eventRequest& req, system_eventResponse& res);
    bool SoundEvent(system_eventRequest& req, system_eventResponse& res);
    void goingHomeStatus(const std_msgs::Int32::ConstPtr& msg);

    void UpdatePatrolState(int state);

    std::vector<bool> mPlayingFlagVec;               
    std::queue<SYSEVT_T> mSoundQue;          
  

    std::thread mPlayThread;
    bool mRunning = false;
    mutex mMutex;

    ros::ServiceServer mSysEventSvr;
    shared_ptr<ros::NodeHandle> mNodeHandle;
    ros::Subscriber mWiFiSub;
    ros::Subscriber mPlayAudioSub;
    ros::Subscriber mPatrolStatsuSub;
    ros::Subscriber mBatteryStatus;
     ros::Subscriber mGoingHomeStatusSub;
    ros:: ServiceClient mGetNavStatusClient;   

    ros::Publisher mSysEvtPub;
    ros::Publisher mPatrolStatusPub;
    
    int mPrevPercentage;
    int mCurrentPercentage;
    int mChargingStatus;
    int mPrevChargingStatus;
    int mPatrolState;                              
};
} // namespace roller_eye
