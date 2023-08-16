#include<thread>
#include<string>
#include<fstream>
#include<cstdlib>
#include <sys/select.h>
#include<linux/input.h>
#include"ros/ros.h"
#include"roller_eye/plog_plus.h"
#include"roller_eye/status.h"
#include"roller_eye/wifi_switch_mode.h"
#include"roller_eye/SensorBase.h"
#include"roller_eye/plt_assert.h"
#include"roller_eye/system_define.h"
#include"roller_eye/plt_tools.h"
#include"roller_eye/device_interface.h"
#include "roller_eye/sound_effects_mgr.h"
#include "roller_eye/system_event.h"
#include "roller_eye/sound_effects_mgr.h"
#include "roller_eye/util_class.h"
#include "roller_eye/wifi_config_info.h"
#include "roller_eye/plt_config.h"
#include "roller_eye/led_all_on.h"
#include "roller_eye/nav_low_bat.h"
#include "zlog.h"

using namespace std;
using namespace roller_eye;

#define USE_BATTERY_STATUS_WATCHDOG		0		///<<< 1

#define UI_NODE_TAG "UI"
//#define WIFI_RETRY_TIME             30
#define WIFI_RETRY_TIME             0
#define POWER_KEY_LONG_PRESS_TIME    (2000)
#define RESET_KEY_LONG_PRESS_TIME    (6000)
#define WIFI_KEY_LONG_PRESS_TIME    (200)

class InitDZLog{
    public:
    InitDZLog()
    {
    	log_t arg = {
			confpath:	"/var/roller_eye/config/log/" UI_NODE_TAG ".cfg",
			levelpath:	"/var/roller_eye/config/log/log.level",
			logpath:	"/var/log/" UI_NODE_TAG ".log",
			cname:		UI_NODE_TAG
		};
		if(0 != dzlogInit(&arg,2)){
			printf("%s log int error.\r\n",UI_NODE_TAG);
		}

    }
    ~InitDZLog()
    {
		dzlogfInit();
    }
} ;

static int s_wifi_mode=status::WIFI_MODE_STA;

static int switchWifiMode(int mode,ros::ServiceClient &client, int kill)
{
    wifi_switch_mode sw;
    sw.request.kill     = kill;
    sw.request.mode=mode;
    return (!client.call(sw)?-1:0);
}

class KeyProcess{
public:
    KeyProcess(ros::NodeHandle &n)
    {
        mSwitch=n.serviceClient<wifi_switch_mode>("WiFiNode/switch_wifi");
        mSysEvtPub = n.advertise<std_msgs::Int32>("/system_event",1);
        mSysEvtClient=n.serviceClient<system_event>("/system_event");
        mConfigNetPub = n.advertise<wifi_config_info>("/WiFiNode/configNet",1);

        mPowerFD=sensor_open_input(POWER_KEY_TYPE.c_str());
        mWiFiFD=sensor_open_input(WIFI_KEY_TYPE.c_str());
        mResetFD = sensor_open_input(RESET_KEY_TYPE.c_str());

        plt_assert(mPowerFD>=0 && mWiFiFD>=0 && mResetFD >= 0);
        mLoop=thread(&KeyProcess::keyLoop,this);
    }
    ~KeyProcess()
    {
        close(mPowerFD);
        close(mWiFiFD);
        close(mResetFD);
        //mLoop.join();
    }
private:
    void keyLoop()
    {
        fd_set fds;
        int maxFD = std::max(std::max(mPowerFD, mWiFiFD), mResetFD) + 1;

        while(true)
        {
            FD_ZERO(&fds);
            FD_SET(mPowerFD,&fds);
            FD_SET(mWiFiFD,&fds);
            FD_SET(mResetFD,&fds);
            if(select(maxFD,&fds,NULL,NULL,NULL)<0){
                plt_assert(errno==EINTR);
                continue;
            }
            if(FD_ISSET(mPowerFD,&fds)){
                processPowerKey();
            }
            if(FD_ISSET(mWiFiFD,&fds)){
                processWiFiKey();
            }

            if(FD_ISSET(mResetFD,&fds)){
              processResetKey();
            }
        }
    }
    void processPowerKey()
    {
        struct input_event ev;
        if(read(mPowerFD,&ev,sizeof(ev))!=sizeof(ev)){
            return;
        }
        if(ev.type==0){
            return;
        }
        if(ev.value==1){
            mLastPowerTime=ev.time;
        }else{
            int t=plt_timeval_diff_ms(&ev.time,&mLastPowerTime);
            if(t>=POWER_KEY_LONG_PRESS_TIME){
                system_event evt;
                evt.request.event = static_cast<int32_t> (SYSEVT_POWEROFF);
                mSysEvtClient.call(evt);
                system(CMD_PREFIX"poweroff");
            }else{
                PLOG_DEBUG(UI_NODE_TAG,"short press time,%d ms",t);
            }
        }
    }
    void processWiFiKey()
    {
        struct input_event ev;
        PLOG_DEBUG(UI_NODE_TAG,"call processWiFiKey");
        if(read(mWiFiFD,&ev,sizeof(ev))!=sizeof(ev)){
            return;
        }
        PLOG_DEBUG(UI_NODE_TAG,"processWiFiKey type:%d code:%d value:%d",ev.type, ev.code, ev.value);
         if(ev.code!=217 || ev.type==0){
            return;
        }
        static int nCnt = 1;
        static bool bPress = false;
        if(ev.value==1){
            bPress = true;
            mLastWifiTime=ev.time;
        }else if (bPress){
            bPress = false;
             int t=plt_timeval_diff_ms(&ev.time,&mLastWifiTime);
             if(t>=WIFI_KEY_LONG_PRESS_TIME){
                PLOG_DEBUG(UI_NODE_TAG,"wifi long press time,%d ms",t);
                int ret;
                std_msgs::Int32 event;
                // ROS_INFO("switch wifi %d",nCnt++);
                if(s_wifi_mode==status::WIFI_MODE_STA){
                    wifi_config_info info;
                    info.cmd = WIFI_CONFIG_CANCEL;
                    mConfigNetPub.publish(info);

                    // event.data = SYSEVT_AP2STA;
                    // mSysEvtPub.publish(event);

                    ret=switchWifiMode(status::WIFI_MODE_AP,mSwitch, 1);
                    if (0==ret){
                        // ROS_INFO("processWiFiKey addWiFi succ");
                        event.data = SYSEVT_AP2STA;
                        mSysEvtPub.publish(event);
                    }
                }else{
                    // event.data = SYSEVT_STA2AP;
                    // mSysEvtPub.publish(event);
                    vector<string> wifi;
                    PltConfig::getInstance()->getWiFis(wifi);
                    if (wifi.empty()){
                        return;
                    }
                    //sw.request.kill     = 2;
                    //system(CMD_PREFIX"killall wifi_start_ap.sh");
                    ret=switchWifiMode(status::WIFI_MODE_STA,mSwitch, 2);
                    if (0==ret){
                        // ROS_INFO("switch wifi sta succ");
                        event.data = SYSEVT_STA2AP;
                        mSysEvtPub.publish(event);
                    }
                }

                if(ret<0){
                    PLOG_ERROR(UI_NODE_TAG,"switch wifi fail\n");
                }
             }else{
                PLOG_DEBUG(UI_NODE_TAG,"wifi short press time,%d ms",t);
             }
        }

    }

    void doReset(){
      std_msgs::Int32 event;
      event.data = SYSEVT_REST_TO_FACTORY;
      mSysEvtPub.publish(event);

      if(!detectProcessIsExited("scout_reset2.sh")){
        /************start add by ltl 2021-07-05***************/
        //update sn from vendor storage
        string sn;
        PltConfig::getInstance()->getSN(sn);
        if (sn.length() > 10){
            PltConfig::getInstance()->updateSN(sn);
        }
        /************end add by ltl 2021-07-05***************/

        string cmd = "/usr/local/bin/scout_reset.sh &";
        system(cmd.c_str());
      }
    }

    void processResetKey(){
      struct input_event ev;
      if(read(mResetFD, &ev,sizeof(ev)) != sizeof(ev)){
        return;
      }

      if(ev.type == 0){
        return;
      }

      if(ev.value == 1){
        mLastResetTime = ev.time;
      }
      else{
        int t = plt_timeval_diff_ms(&ev.time, &mLastResetTime);
        if(t >= RESET_KEY_LONG_PRESS_TIME){
          PLOG_DEBUG(UI_NODE_TAG, "handle processResetKey");
          doReset();
        }
        else{
          PLOG_DEBUG(UI_NODE_TAG, "short reset press time, %d ms",t);
        }
      }
    }

    thread mLoop;
    ros::ServiceClient mSwitch;
    ros:: ServiceClient mSysEvtClient;
    ros::Publisher  mSysEvtPub;
    ros::Publisher mConfigNetPub;
    int mPowerFD;
    struct timeval mLastPowerTime;
    struct timeval mLastWifiTime;
    int mWiFiFD;
    struct timeval mLastResetTime;
    int mResetFD;

    const string POWER_KEY_TYPE="rk8xx_pwrkey";
    const string WIFI_KEY_TYPE="rk29-keypad";
    const string RESET_KEY_TYPE = "adc-keys";
};

enum{
    WIFI_LED_MODE_AP=0,
    WIFI_LED_MODE_STA
};
enum{
    WIFI_LED_ON=0,
    WIFI_LED_OFF
};
class LEDProcess{
public:
    LEDProcess(ros::NodeHandle &n):mDisconnectCount(0),mWrongKey(false),mBling(false)
    {
        mWiFiSub=n.subscribe("WiFiNode/status", 10, &LEDProcess::wifiStatusCallback,this);
        mWiFiDeamon=n.createTimer(ros::Duration(1.0),&LEDProcess::timerCallback,this);
        mWiFiDeamon.stop();
        wifiLEDStop();
    }
    void wifiLEDOk(int mode)
    {
        stopBling();
        wifiLEDControl(mode,WIFI_LED_ON);
        wifiLEDControl((mode+1)&0x1,WIFI_LED_OFF);
    }
    void wifiLEDConnetting(int mode)
    {
        wifiLEDStop();
        startBling(mode,200);
    }
    void wifiLEDDisconnet(int mode)
    {
        wifiLEDStop();
        startBling(mode,1000);
    }
    void wifiLEDStop()
    {
        stopBling();
        wifiLEDControl(WIFI_LED_MODE_AP,WIFI_LED_OFF);
        wifiLEDControl(WIFI_LED_MODE_STA,WIFI_LED_OFF);
    }
private:
    void wifiStatusCallback(const statusConstPtr &s)
    {
        mWiFiMode=s->status[0];
        mWiFiStatus=s->status[1];
        s_wifi_mode=mWiFiMode;
        PLOG_INFO(UI_NODE_TAG,"wifi status:mode=%d,status=%d\n",mWiFiMode,mWiFiStatus);
        switch (mWiFiStatus)
        {
        case status::WIFI_STATUS_STOP:
            wifiLEDStop();
            //mWiFiDeamon.start();
            break;
        case status::WIFI_STATUS_CONNECTING:
            mWrongKey=false;
            wifiLEDConnetting(mWiFiMode==status::WIFI_MODE_STA?WIFI_LED_MODE_STA:WIFI_LED_MODE_AP);
            break;
        case status::WIFI_STATUS_CONNECTED:
            wifiLEDOk(mWiFiMode==status::WIFI_MODE_STA?WIFI_LED_MODE_STA:WIFI_LED_MODE_AP);
            break;
        case status::WIFI_STATUS_WRONG_KEY:
            mWrongKey=true;
        case status::WIFI_STATUS_DISCONNECT:
        case status::WIFI_STATUS_CONN_FAIL:
             wifiLEDDisconnet(mWiFiMode==status::WIFI_MODE_STA?WIFI_LED_MODE_STA:WIFI_LED_MODE_AP);
        default:
            break;
        }
    }
    void timerCallback(const ros::TimerEvent& evt)
    {
        // PLOG_DEBUG(UI_NODE_TAG,"WiFi Deamon timer,counter=%d",mDisconnectCount);
        // if(mWiFiStatus!=status::WIFI_STATUS_STOP){
        //     mWiFiDeamon.stop();
        //     mDisconnectCount=0;
        // }else{
        //     if(++mDisconnectCount>WIFI_RETRY_TIME){
        //         mDisconnectCount=0;
        //         mWiFiDeamon.stop();
        //         /*if(!mWrongKey)*/{
        //             switchWifiMode(mWiFiMode,mSwitch, 0);
        //         }
        //     }
        // }
    }
    void stopBling()
    {
        if(mBling)
        {
            mBling=false;
            mBlingThread.join();
        }
    }
    void startBling(int id,int time)
    {
        if(!mBling){
            mBling=true;
            mBlingThread=thread([this](int id,int time){
                bool flag=false;
                struct timeval tm;
                while(mBling){
                    wifiLEDControl(id,flag?WIFI_LED_ON:WIFI_LED_OFF);
                    flag=!flag;

                    gettimeofday(&tm, nullptr);
                    uint64_t start = tm.tv_sec * 1000000 + tm.tv_usec;
                    uint64_t now = start;
                    while(mBling && now<start+time*1000){
                        usleep(20*1000);
                        gettimeofday(&tm, nullptr);
                        now = tm.tv_sec * 1000000 + tm.tv_usec;
                    }
                    //usleep(time*1000);
                }
            },id,time);
        }
    }
    void wifiLEDControl(int id,int flag)
    {
        static const char* ids[]={"gpio00_","gpio01_"};
        //static const char* flags[]={"0","1"};
        static const char* flags[]={"1", "0"};
        char buff[16];

        PLOG_ERROR(UI_NODE_TAG,"wifiLEDControl %d\n",id);
        snprintf(buff,sizeof(buff),"%s%s\n",ids[id],flags[flag]);
        ofstream of(LED_PATH);
        if(!of){
            PLOG_ERROR(UI_NODE_TAG,"can't not open %s\n",LED_PATH.c_str());
            return;
        }
        of<<buff<<std::flush;
    }
    ros::Subscriber mWiFiSub;
    ros::Timer mWiFiDeamon;
    int mDisconnectCount;
    int mWiFiMode;
    int mWiFiStatus;
    bool mWrongKey;
    const string LED_PATH="/proc/driver/gpioctl";
    bool mBling;
    thread mBlingThread;
};
class PowerStatusProcess{
public:
    PowerStatusProcess(ros::NodeHandle &n):
    mBlingStage(0),
    mCurrentPercentage(100),
    mBistStart(0),
    mPrevPercentage(100)
    {
        mMinBat = LOW_BATTERY_PER;
        PltConfig::getInstance()->getMinBat(mMinBat);
        mPrevPercentage = mPrevPercentage>mMinBat?mPrevPercentage:(mMinBat+1);

        mLedSrv = n.advertiseService("/led_all_on",&PowerStatusProcess::ledOn,this);
        mBatteryStatus=n.subscribe("SensorNode/simple_battery_status", 10, &PowerStatusProcess::batteryStatusCallback,this);
		mNavLowBatClient = n.serviceClient<nav_low_bat>("/nav_low_bat");

        //rmwei
        if (!mSysEventSub){
            mSysEventSub = n.subscribe("/system_event", 10,
						&PowerStatusProcess::SysEventCallback, this);
        }
        mTimer=n.createTimer(ros::Duration(0.5),&PowerStatusProcess::timerCallback,this);
        mTimer.stop();
    }
private:
    void SysEventCallback(const std_msgs::Int32::ConstPtr& msg)
    {
        SYSEVT_T evt = static_cast<SYSEVT_T>(msg->data);

        if(SYSEVT_BIST_START == evt){
        	PLOG_ERROR(UI_NODE_TAG,"SYSEVT_BIST_START .");
			mBistStart = 1;
        }
    }

    void setPowerPercentage(int per)
    {
        //PLOG_DEBUG(UI_NODE_TAG,"set power percentage %d\n",per);

        int status[4]={0};

        if(per>0){
            status[0]=1;
        }
        if(per>25){
            status[1]=1;
        }
        if(per>50){
            status[2]=1;
        }
        if(per>75){
            status[3]=1;
        }
        for(int i=0;i<4;i++){
        	if(0 == mBistStart){
            	set_power_led_status(i,status[i]);
        	}
        }
    }
    void batteryStatusCallback(const statusConstPtr &s)
    {
        mCurrentPercentage=s->status[1];
        if(s->status[0]==status::BATTERY_CHARGING){
            mTimer.start();
        }else{
            mTimer.stop();
            mBlingStage=0;
            setPowerPercentage(s->status[1]);
            int minBat = LOW_BATTERY_PER/4;
             PltConfig::getInstance()->getMinBatForPowerOff(minBat);
             if (s->status[1] <= minBat){  // low battery auto poweroff
                system(CMD_PREFIX"poweroff");
            }
        }

	#if USE_BATTERY_STATUS_WATCHDOG
		wCnt = 0;
		mTimer.start();
    	if(s->status[0]==status::BATTERY_CHARGING)
			mCharging = true;
		else
			mCharging = false;
	#endif

        static bool bNotify = false;
        PLOG_INFO(UI_NODE_TAG,"s->status[2]: %d bNotify:%d mPrevPercentage:%d mMinBat:%d mCurrentPercentage:%d\n",
                                 s->status[2], bNotify, mPrevPercentage, mMinBat, mCurrentPercentage);
		if (s->status[2]){
            bNotify = false;
			mPrevPercentage = 100>mMinBat ?100:(mMinBat+1);
		}else{
            if (!bNotify &&
                   mPrevPercentage>mMinBat  &&
                   mCurrentPercentage<=mMinBat){    ///<<< Fix Backup not start while batt level down to mMinBat level
                nav_low_bat nlb;
                PLOG_INFO(UI_NODE_TAG, "call battery low!");
                bNotify = mNavLowBatClient.call(nlb);
                if (bNotify){
        	        mPrevPercentage = mCurrentPercentage;
                }
            }else{
        	    mPrevPercentage = mCurrentPercentage;
            }
		}
    }
    void timerCallback(const ros::TimerEvent& evt)
    {
#if USE_BATTERY_STATUS_WATCHDOG
		 wCnt ++;
		 if(0 == (wCnt% 360 ))  ///<<< no update for  more than 3 minutes
		 {
			///if(mBatteryStatus) mBatteryStatus.shutdown();
			PLOG_FATAL(UI_NODE_TAG,"Restart SensorNode due to no battery status update (cnt = %d)!!! ",wCnt);
			system("sudo killall sensors_node");	///<<< try to restart SensorNode
			///sleep(2);
			///mBatteryStatus=n.subscribe("SensorNode/simple_battery_status", 10, &PowerStatusProcess::batteryStatusCallback,this);
		 }
		 if (false == mCharging)
		 	return;
#endif
        setPowerPercentage(mBlingStage);
        if(mBlingStage>mCurrentPercentage){
            mBlingStage=0;
        }else{
             mBlingStage+=25;
        }
    }

    bool ledOn(led_all_onRequest& req,led_all_onResponse& res)
    {
        for(int i=0;i<4;i++){
            set_power_led_status(i,1);
        }
        wifiLEDControl(WIFI_LED_MODE_AP, WIFI_LED_ON);
        wifiLEDControl(WIFI_LED_MODE_STA, WIFI_LED_ON);
        return true;
    }
    void wifiLEDControl(int id,int flag)
    {
        static const char* ids[]={"gpio00_","gpio01_"};
        //static const char* flags[]={"0","1"};
        static const char* flags[]={"1", "0"};
        char buff[16];

        PLOG_ERROR(UI_NODE_TAG,"wifiLEDControl %d\n",id);
        snprintf(buff,sizeof(buff),"%s%s\n",ids[id],flags[flag]);
        ofstream of(LED_PATH);
        if(!of){
            PLOG_ERROR(UI_NODE_TAG,"can't not open %s\n",LED_PATH.c_str());
            return;
        }
        of<<buff<<std::flush;
    }
    const string LED_PATH="/proc/driver/gpioctl";
    ros::Subscriber mBatteryStatus;
    ros::Subscriber mSysEventSub;
    ros::ServiceClient  mNavLowBatClient;
    ros::Timer mTimer;
    ros::ServiceServer mLedSrv;
    int mBlingStage;
    int mCurrentPercentage;
    int mBistStart = 0;
#if USE_BATTERY_STATUS_WATCHDOG
    int wCnt = 0;
	bool mCharging = false;
#endif
    int mMinBat;
    int mPrevPercentage;
};

int main(int argc, char **argv)
{
    InitDZLog dzLog;
  ros::init(argc, argv, "UINode");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,UI_NODE_DEBUG_LEVEL);
  ros::NodeHandle n("");
  KeyProcess key(n);
  LEDProcess led(n);
  PowerStatusProcess power(n);
  ros::spin();
  return 0;
}