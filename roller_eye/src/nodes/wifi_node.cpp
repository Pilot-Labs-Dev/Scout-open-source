#include"ros/ros.h"
#include"roller_eye/plog_plus.h"
#include"roller_eye/status.h"
#include "roller_eye/wifi_switch_ssid_key.h"
#include"roller_eye/wifi_switch_mode.h"
#include"roller_eye/wifi_scan_list.h"
#include"roller_eye/wifi_ops.h"
#include"roller_eye/status_publisher.h"
#include"roller_eye/system_define.h"
#include "roller_eye/wifi_config_info.h"
#include "roller_eye/wifi_get_mode.h"
#include <thread>
#include "zlog.h"

using namespace roller_eye;

#define WIFI_NODE_TAG "WiFiNode"
class InitDZLog{
    public:
    InitDZLog()
    {
    	log_t arg = {
			confpath:	"/var/roller_eye/config/log/" WIFI_NODE_TAG ".cfg",
			levelpath:	"/var/roller_eye/config/log/" WIFI_NODE_TAG ".level",
            //levelpath: "/var/roller_eye/config/wifi_log.level",
			logpath:	"/var/log/node/" WIFI_NODE_TAG ".log",
			cname:		WIFI_NODE_TAG
		};
        system("rm /var/roller_eye/config/log/" WIFI_NODE_TAG ".cfg");
        system("rm /var/roller_eye/config/log/" WIFI_NODE_TAG ".level");
		if(0 != dzlogInit(&arg,4)){
			printf("%s log int error.\r\n",WIFI_NODE_TAG);
		}
        system("echo 20 > /var/roller_eye/config/log/" WIFI_NODE_TAG ".level");
		zlog_category_t *zc = zlog_get_category(WIFI_NODE_TAG);
		zlog_level_switch(zc,20);

    }
    ~InitDZLog()
    {
		dzlogfInit();
    }
} ;

struct timeval gScanCmdTm;
vector<WiFiInFo> gvWifis;
mutex gMutex;
class WiFiNodeListener:public WiFiOps::Eventlistener{
public:
    WiFiNodeListener(ros::NodeHandle &n):
    mPub(n)
    {

    }
    ~WiFiNodeListener()
    {

    }

    void onEvent(int mode,int status)
    {
        vector<int32_t> s;

        if(mapMode(mode)<0||mapStatus(status)<0){
            PLOG_ERROR(WIFI_TAG,"unkonw event");
            return ;
        }

        s.push_back(mode);
        s.push_back(status);
        mPub.pubStatus(s);
    }

    static int mapMode(int &mode)
    {
        if(mode==WiFiOps::WIFI_MODE_AP){
            mode=roller_eye::status::WIFI_MODE_AP;
        }else if(mode==WiFiOps::WIFI_MODE_STA){
            mode=roller_eye::status::WIFI_MODE_STA;
        }else{
            return -1;
        }
        return 0;
    }
    
    static int mapStatus(int &status)
    {
        int ret=0;
        switch (status)
        {
        case WiFiOps::WIFI_EVENT_CONNECTED:
            status=roller_eye::status::WIFI_STATUS_CONNECTED;
            //WiFiNode::publish_config_net(WIFI_CONFIG_SUCCESS);       
            PLOG_INFO(WIFI_TAG,"WIFI_EVENT_CONNECTED");   
            break;
        case WiFiOps::WIFI_EVENT_DISCONNECT:
            status=roller_eye::status::WIFI_STATUS_DISCONNECT;
            PLOG_INFO(WIFI_TAG,"WIFI_EVENT_DISCONNECT");   
            break;
        case WiFiOps::WIFI_EVENT_CONNECTING:
             status=roller_eye::status::WIFI_STATUS_CONNECTING;
            PLOG_INFO(WIFI_TAG,"WIFI_EVENT_CONNECTING");   
            break;
        case WiFiOps::WIFI_EVENT_WRONG_KEY:
            status=roller_eye::status::WIFI_STATUS_WRONG_KEY;
            PLOG_INFO(WIFI_TAG,"WIFI_EVENT_WRONG_KEY");   
            break;
        case WiFiOps::WIFI_EVENT_CONN_FAIL:
            status=roller_eye::status::WIFI_STATUS_CONN_FAIL;
            PLOG_INFO(WIFI_TAG,"WIFI_EVENT_CONN_FAIL");        
            break;
        case WiFiOps::WIFI_EVENT_STOP:
            status=roller_eye::status::WIFI_STATUS_STOP;
            PLOG_INFO(WIFI_TAG,"WIFI_EVENT_STOP");   
            break;
        default:
            ret=-1;
            break;
        }
        return ret;
    }
private:
    StatusPublisher mPub;
};

class WiFiNode{
  public:
    WiFiNode():
    mHandle("~"),
    m_bScanning(false)
    {
        string sn;
        PltConfig::getInstance()->getSN(sn);
        if (sn.length()>10){
            PltConfig::getInstance()->updateSN(sn);
        }        
        mWifiListener=new WiFiNodeListener(mHandle);
        WPAWiFiOps::getInstance()->setListener(mWifiListener);
        mSwitchSsidKey=mHandle.advertiseService("switch_wifi_ssidkey", &WiFiNode::wifi_switch_ssid_key, this);
        mSwitch=mHandle.advertiseService("switch_wifi", &WiFiNode::wifi_switch_mode, this);
        mGetList=mHandle.advertiseService("wifi_scan_list", &WiFiNode::wifi_scan_list, this);
        mGetMode=mHandle.advertiseService("wifi_get_mode", &WiFiNode::wifi_get_mode, this);

        mConfigNetPub = mHandle.advertise<wifi_config_info>("/WiFiNode/configNet",1);

        mConfigNetSub=mHandle.subscribe("/WiFiNode/configNet",1,
                                                                        &WiFiNode::onConfigWifi,this);
        mWifiMonitor=mHandle.subscribe("/WiFiNode/status",10,&WiFiNode::wifiCloudCallback, this);
        m_timer= mHandle.createTimer(ros::Duration(1), 
                                                                           &WiFiNode::timerCallback, this, false, false);  
        m_timer.start();

    }
    ~WiFiNode()
    {

    }
    void run(){
        WPAWiFiOps::getInstance()->run();
    }
    
    void timerCallback(const ros::TimerEvent& event)
    {
        static int counter = 0;
        if (WIFI_OP_SWITCH ==  mWifiOpConfigNet){
            ros::Time curTm = ros::Time::now();
            if (counter++>60){                   
                if (WIFI_OP_SWITCH == mWifiOpConfigNet){
                    mWifiOpConfigNet = WIFI_OP_NONE;
                    WPAWiFiOps::getInstance()->updateWiFis(false);
                }
                switchMode(WPAWiFiOps::getInstance()->getMode());  
            }
        }else{
            counter = 0;
            if (WPAWiFiOps::getInstance()->getMode() == WiFiOps::WIFI_MODE_STA && 
                   status::WIFI_STATUS_CONNECTED == mWifiStatus ){
                    vector<string> vWifis;
                    PltConfig::getInstance()->getWiFis(vWifis);  
                    if (vWifis.size()>=2){
                        char buff[256];
                        get_gateway(buff);  
                        WPAWiFiOps::getInstance()->resetIsSwitch();
                        if (ping_status(buff) !=0 &&
                            WPAWiFiOps::getInstance()->getMode() == WiFiOps::WIFI_MODE_STA ){
                            if (!WPAWiFiOps::getInstance()->isSwitch()){
                                WPAWiFiOps::getInstance()->reconnectWifi();
                            }
                        }
                    }    
            }
        }
    }

    void scanLoop()
    {
        int nCnt = 0;
        m_bScanning = true;
        queue<vector<WiFiInFo>> qvWifis;
        
        while(m_bScanning){
            if (mWifiOpConfigNet != WIFI_OP_NONE){
                WPAWiFiOps::getInstance()->stopScanWifi();
                break;
            }
            if (nCnt++<6){
                vector<WiFiInFo> wifis;
                if (!WPAWiFiOps::getInstance()->scanWiFilist(wifis)){
                    break;
                }
                if (wifis.size() > 0){
                    qvWifis.push(wifis);
                    if (qvWifis.size() > 6){
                        qvWifis.pop();
                    }                
                    gMutex.lock();
                    gvWifis.clear();
                    map<string,bool> mapSsid;
                    for (auto i=0; i<qvWifis.size(); i++){
                        vector<WiFiInFo> wifis = qvWifis.front();
                        qvWifis.pop();
                        qvWifis.push(wifis);
                        for (auto it:wifis){
                            if (mapSsid.find(it.ssid) == mapSsid.end()){
                                gvWifis.push_back(it);
                                mapSsid[it.ssid] = true;
                            }
                        }
                    }
                   gMutex.unlock();
                }
            }else{
                gMutex.lock();
                gvWifis.clear();
                gMutex.unlock();
                queue<vector<WiFiInFo>> empty;
	            swap(empty, qvWifis);
                usleep(200*1000);
                break;
            }
        }
        m_bScanning = false;   
    }

    void publish_config_net(int8_t status)
    {
        wifi_config_info info;
        info.cmd = status;
        info.ssid = mStrSsid;
        mConfigNetPub.publish(info);
    }

    bool wifi_switch_ssid_key(roller_eye::wifi_switch_ssid_keyRequest  &req,roller_eye::wifi_switch_ssid_keyResponse &resp)
    {
        int mode=req.mode;
        if(WiFiNodeListener::mapMode(mode)<0){
            return false;
        }       

        resp.status=WPAWiFiOps::getInstance()->addSSIDKey(mode,req.ssid,req.key)==0?status::PROCESS_OK:status::PROCESS_ERROR;
        if(resp.status!=status::PROCESS_OK){
            PLOG_ERROR(WIFI_TAG,"wifi_switch_ssid_key wifi fail");  
            return false;
        }
        mWifiOpConfigNet = WIFI_OP_SWITCH;     
        switchMode(WiFiOps::WIFI_MODE_STA,  req.ssid,req.key);        
        return true;
    }

    bool wifi_switch_mode(roller_eye::wifi_switch_modeRequest &req,roller_eye::wifi_switch_modeResponse& resp)
    {
        int mode=req.mode;
        if(WiFiNodeListener::mapMode(mode)<0){
            return false;
        }
        WPAWiFiOps::getInstance()->stopScript(req.kill);

        mWifiListener->onEvent(mode,WPAWiFiOps::WIFI_EVENT_CONNECTING);
        WPAWiFiOps::getInstance()->setLastMode(mode);
        resp.status=switchMode(mode)==0?status::PROCESS_OK:status::PROCESS_ERROR;
        if(resp.status!=status::PROCESS_OK){
            PLOG_ERROR(WIFI_TAG,"add wifi fail");
            return false;
        }
        return true;
    }

    bool wifi_scan_list(roller_eye::wifi_scan_listRequest &req,roller_eye::wifi_scan_listResponse& resp)
    {
        gettimeofday(&gScanCmdTm, nullptr);
        
        if (mWifiOpConfigNet !=WIFI_OP_NONE){
            return false;
        }

        if (!m_bScanning){
           std::thread th(&WiFiNode::scanLoop, this);        
           th.detach(); 
        }
        
        int waits=0; 
        while (gvWifis.empty() && waits++<10){
            usleep(1000*1000);
        }

        gMutex.lock();
        if (gvWifis.size() > 0){
             for(auto& wifi:gvWifis){
                wifi_info info;
                info.ssid=wifi.ssid;
                info.quality=wifi.quality;
                info.signal=wifi.signal;
                info.noisy=wifi.noisy;
                info.freq=wifi.freq;
                info.channel=wifi.channel;
                resp.result.push_back(info);
            }
        }
        gMutex.unlock();        

        return true;
    }

    bool wifi_get_mode(roller_eye::wifi_get_modeRequest &req,roller_eye::wifi_get_modeResponse& resp)
    {
        resp.mode = WPAWiFiOps::getInstance()->getMode();  
        return true;
    }
   
    void onConfigWifi(const wifi_config_info& info)
    {
        switch(info.cmd){ 
            case WIFI_CONFIG_START:    
            mWifiOpConfigNet = WIFI_OP_CONFIG;
            PLOG_INFO(WIFI_TAG,"WIFI_CONFIG_START");
            break;
            case WIFI_CONFIG_SUCCESS:    
            mWifiOpConfigNet = WIFI_OP_NONE;
            break;
            case WIFI_CONFIG_FAILURE:    
            mWifiOpConfigNet = WIFI_OP_NONE;   
            WPAWiFiOps::getInstance()->stopScript(1);
            switchMode(WiFiOps::WIFI_MODE_AP);   
            PLOG_INFO(WIFI_TAG,"WIFI_CONFIG_FAILURE");
            break;
            case  WIFI_CONFIG_CANCEL:   //key cancel
            WPAWiFiOps::getInstance()->updateWiFis(false);     
            mWifiOpConfigNet = WIFI_OP_NONE;
            PLOG_INFO(WIFI_TAG,"WIFI_CONFIG_CANCEL");
            break;
            default:
            break;
        }
    }

    void wifiCloudCallback(const statusConstPtr &status)
    {
        mWifiStatus = status->status[1];
        PLOG_INFO(WIFI_TAG,"wifiCloudCallback %d",status->status[1]);
        switch (status->status[1])
        {
        case status::WIFI_STATUS_CONNECTED:
            PLOG_INFO(WIFI_TAG,"WIFI_EVENT_CONNECTED");
            if (WIFI_OP_SWITCH == mWifiOpConfigNet){
                WPAWiFiOps::getInstance()->updateWiFis(true);
                mWifiOpConfigNet = WIFI_OP_NONE;
            }
            break;
        case status::WIFI_STATUS_DISCONNECT:
            PLOG_INFO(WIFI_TAG,"WIFI_EVENT_DISCONNECT");  
            if (WIFI_OP_SWITCH == mWifiOpConfigNet){
                WPAWiFiOps::getInstance()->updateWiFis(false);
                mWifiOpConfigNet = WIFI_OP_NONE;
            }
            switchMode(WPAWiFiOps::getInstance()->getMode());  
            break;
        case status::WIFI_STATUS_CONNECTING:
            PLOG_INFO(WIFI_TAG,"WIFI_EVENT_CONNECTING");
            break;
        case status::WIFI_STATUS_WRONG_KEY:
            WPAWiFiOps::getInstance()->updateWiFis(false);
            if (WIFI_OP_CONFIG ==  mWifiOpConfigNet){              
                publish_config_net(WIFI_CONFIG_FAILURE);      
            }  else{
                if (WIFI_OP_SWITCH == mWifiOpConfigNet){
                    WPAWiFiOps::getInstance()->updateWiFis(false);
                    mWifiOpConfigNet = WIFI_OP_NONE;
                }

                switchMode(WPAWiFiOps::getInstance()->getMode());   
            }
            break;
        case status::WIFI_STATUS_CONN_FAIL:  
            PLOG_INFO(WIFI_TAG,"WIFI_EVENT_CONN_FAIL");  
            if (WIFI_OP_CONFIG ==  mWifiOpConfigNet){
                switchMode(WiFiOps::WIFI_MODE_STA);   
            }else{
                if (WIFI_OP_SWITCH == mWifiOpConfigNet){
                    WPAWiFiOps::getInstance()->updateWiFis(false);
                    mWifiOpConfigNet = WIFI_OP_NONE;
                }
                switchMode(WPAWiFiOps::getInstance()->getMode());  
            }
            break;
        case status::WIFI_STATUS_STOP:
            PLOG_INFO(WIFI_TAG,"WIFI_EVENT_STOP");
            break;
        default:
            break;
        }
    }
    void stopScan()
    {
        m_bScanning = false;
        WPAWiFiOps::getInstance()->stopScanWifi();
    }
    int switchMode(int mode)
    {
        stopScan();
        return WPAWiFiOps::getInstance()->switchMode(mode);  
    }
    int switchMode(int mode,string& ssid,string& key)
    {
       stopScan();
       WPAWiFiOps::getInstance()->switchMode(WiFiOps::WIFI_MODE_STA,  ssid, key);     
    }
  private:    
    ros::NodeHandle mHandle;  
    ros::ServiceServer mAdd;
    ros::ServiceServer mSwitchSsidKey;
    ros::ServiceServer mConfigSsidResult;    
    ros::ServiceServer mSwitch;
    ros::ServiceServer mGetList;
    ros::ServiceServer mGetMode;
    ros::Publisher mConfigNetPub;   
    ros::Subscriber mConfigNetSub;
    ros::Subscriber mWifiMonitor;
    ros::Timer m_timer;
    int mWifiStatus;
    string mStrSsid;                                     //ssid
    WIFI_OP_T mWifiOpConfigNet = WIFI_OP_NONE;        
    WiFiNodeListener* mWifiListener;
    bool m_bScanning;
};

int main(int argc, char **argv)
{
  InitDZLog dzLog;
  ros::init(argc, argv, "WiFiNode");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,WIFI_NODE_DEBUG_LEVEL);
  WiFiNode node;
  node.run();
  ros::spin();
  return 0;
}
