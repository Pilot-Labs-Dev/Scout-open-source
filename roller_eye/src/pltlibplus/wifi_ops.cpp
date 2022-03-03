#include<thread>
#include<fstream>
#include<stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/wait.h>
#include<roller_eye/wifi_ops.h>
#include"roller_eye/plog_plus.h"
#include "roller_eye/plt_tools.h"

namespace roller_eye{
const char* WIFI_TAG="wifi";

class CompGreater 
{ 
public: 
    bool operator ()(const WiFiInFo& info1, const WiFiInFo& info2) 
    { 
        return info1.signal > info2.signal; 
    }   
}; 

WiFiOps::Eventlistener::Eventlistener()
{

}
WiFiOps::Eventlistener::~Eventlistener()
{

}

void WiFiOps::run()
{
    if(mRuning.test_and_set()){
        PLOG_WARN(WIFI_TAG,"WiFiOps Is running...\n");
        return;
    }
    string sn;
    PltConfig::getInstance()->getSN(sn);
    if (sn.length()<10){
        PLOG_ERROR(WIFI_TAG,"no sn don't config wifi..\n");
        return;
    }

    thread th([](WiFiOps* ops){
        thread loop([](WiFiOps* ops){
            ops->runLoop();
        },ops);

        while(true){
            ops->cmdProc();
            usleep(20*1000);
        }
        loop.join();
        ops->mRuning.clear();
    },this);
    th.detach();
}

void WiFiOps::cmdProc()
{
    auto cmd=make_shared<Ops>();
    mMutex.lock();
    *cmd = mOps;
    mOps.ssid.erase();
    mOps.key.erase();
    mOps.ops = WIFI_OPS_NONE;
    mMutex.unlock();
    if (cmd->ops != WIFI_OPS_NONE){
        handleCmd(cmd);
    }
}

int WiFiOps::stopWiFi()
{
     auto cmd=make_shared<Ops>();
     cmd->ops=WIFI_OPS_STOP;
     return mOpsQueue.pushNoblock(cmd);
}
int WiFiOps::addSSIDKey(int mode,string& ssid,string& key)
{
    lock_guard<mutex> lock(mMutex);
    mOps.ssid = ssid;
    mOps.key  = key;
    mOps.ops     = WIFI_OPS_ADD;
    mOps.mode = mode;
    return 0;
}
void WiFiOps::clearOps()
{
    mMutex.lock();
    mOps.ssid.erase();
    mOps.key.erase();
    mOps.ops = WIFI_OPS_NONE;
    mMutex.unlock();
}
int WiFiOps::switchMode(int mode)
{   
    lock_guard<mutex> lock(mMutex);
    switch(mOps.ops){
        case WIFI_OPS_SWITCH:
            mOps.ssid.erase();
            mOps.key.erase();
        break;
        case WIFI_OPS_ADD:
            if (WIFI_MODE_STA != mOps.mode){
                mOps.ssid.erase();
                mOps.key.erase();
            }
        break;
        case WIFI_OPS_STOP:
        case WIFI_OPS_NONE:
        break;
    }
    mOps.ops     = WIFI_OPS_SWITCH;
    mOps.mode = mode;
    setIsSwitch();
    ROS_INFO("switchMode %d %s:%d %s\n",mode, __FILE__, __LINE__,__FUNCTION__);
    return 0;
}

int WiFiOps::switchMode(int mode,string& ssid,string& key)
{    
    lock_guard<mutex> lock(mMutex);
    mOps.ssid = ssid;
    mOps.key  = key;
    mOps.ops     = WIFI_OPS_SWITCH;
    mOps.mode = mode;

    setIsSwitch();
    ROS_INFO("switchMode %d %s %s %s:%d %s\n",mode, ssid.c_str(), key.c_str(),  
                              __FILE__, __LINE__,__FUNCTION__);
    return 0;
}

void WiFiOps::setListener(Eventlistener* lisnter)
{
    mListener=lisnter;   
}
WiFiOps::WiFiOps():
mListener(nullptr),
mWiFiMode(WIFI_MODE_UNKOWN),
mOpsQueue(MAX_WIFI_CMD_SIZE),
mRuning(false)
{

}
WiFiOps::~WiFiOps()
{
    if(mListener!=nullptr){
        delete mListener;
    }  
}
 WPAWiFiOps::WPAWiFiOps():mStaReady(false), mLastestMode(WIFI_MODE_STA), mRunScripting(false),
 mRunAnaly(false)
 {
     mConfig=PltConfig::getInstance();
     mConfig->getWiFis(mWiFis);       
 }
WPAWiFiOps::~WPAWiFiOps()
{

}

void WPAWiFiOps::updateWiFis(const string& ssid,const string& key)
{
    mSsid = ssid;
    mKey  = key; 
    ROS_INFO("WiFi updateWiFis %s %s %s:%d %s\n",ssid.c_str(), key.c_str(), __FILE__ ,__LINE__,__FUNCTION__);
    updateWiFis();
}
void WPAWiFiOps::updateWiFis(bool bUpdate)
{
    if (mSsid.empty()){
        return;
    }
    
    string ssid = mSsid;
    string key = mKey;  
    ROS_INFO("WiFi updateWiFis %s %s %s:%d %s\n",ssid.c_str(), key.c_str(), __FILE__ ,__LINE__,__FUNCTION__);
      
    if(!bUpdate){
        mSsid.erase();
        mKey.erase();    
        return;
    }

    bool bExist = false;
    if (!mSsid.empty()){
        for (int i=0; i<static_cast<int>(mWiFis.size()); i+=2){
            if (mSsid == mWiFis[i]){
                bExist = true;
                break;
            }
        }   
    }
    
    if (!bExist){
        mWiFis.push_back(ssid);
        mWiFis.push_back(key);
        ROS_INFO("WiFi updateWiFis %s %s %s:%d %s\n",ssid.c_str(), key.c_str(), __FILE__ ,__LINE__,__FUNCTION__);
        mConfig->updateWiFis(mWiFis);
    }

    mSsid.erase();
    mKey.erase();    
}

bool WPAWiFiOps::isDisconnect()
{
    char status[256]={0};   

    int rval = 0;
    string cmd = "iwconfig wlan0";
    FILE* fcmd=popen(cmd.c_str(),"r");
    if(fcmd==NULL){
        PLOG_ERROR(APP_NODE_TAG,"Open %s\n", cmd.c_str());
        return true;     
    }   

    char* t = NULL;
    while (!feof(fcmd)){        
        if (fgets(status,sizeof(status),fcmd) == NULL){
            usleep(10*1000);
            continue;
        }
        if((t=const_cast<char*>(strrchr(status,'\n')))==NULL){
            PLOG_ERROR(WIFI_TAG,"%s %d Line Over Flow or reach EOF\n", __FILE__, __LINE__);
            continue;
        }else if (NULL != strstr(status, "Access Point: Not-Associated")) {
            return true;
        }
    }
    fclose(fcmd);   
    return false;
}

void WPAWiFiOps::runLoop()
{
    char status[512]={0};   

    system(CMD_PREFIX"killall wpa_cli");
    sleep(3);

    ROS_INFO("WiFi runLoop switchMode %s:%d %s\n",__FILE__, __LINE__,__FUNCTION__);
    switchMode(mWiFis.empty()?WIFI_MODE_AP:WIFI_MODE_STA);

    while(true){
        if (mRunAnaly){  
            if (isDisconnect()){
                stopScript(1);
                mListener->onEvent(WIFI_MODE_STA,WIFI_EVENT_DISCONNECT);
                switchMode(mWiFis.empty()?WIFI_MODE_AP:WIFI_MODE_STA);
            }   
        }           
        usleep(200*1000);
    }
    ROS_INFO("quit WPAWiFiOps::runLoop() ");
}

void WPAWiFiOps::stopScript(int kill)
{
        switch(kill){
            case 1:
            mRunAnaly = false;
            system(CMD_PREFIX"killall wifi_start_sta.sh");
            mRunScripting = false;
            break;
            case 2:system(CMD_PREFIX"killall wifi_start_ap.sh");
            mRunAnaly = false;
            mRunScripting = false;
            break;
        }
}

int WPAWiFiOps::runScript(const string& cmd)
{
      char status[1024]={0};   

//#define RETURNED_VALUE "__returned_value_"
#define EXEC_SUCC "start wifi ok"
#define EXEC_FAIL "start wifi fail"

    int rval = -1;
    FILE* fcmd=popen(cmd.c_str(),"r");
    if(fcmd==NULL){
        PLOG_ERROR(WIFI_TAG,"Open %s\n", cmd.c_str());
        return rval;     
    }   

    lock_guard<mutex> lock(mMutex);
    int fd = fileno(fcmd);
    int flags;
    flags = fcntl(fd, F_GETFL, 0);
    flags |= O_NONBLOCK;
    fcntl(fd, F_SETFL, flags);
    mRunScripting = true;
    
    char* t = NULL;
    while (mRunScripting && !feof(fcmd)){        
        if (fgets(status,sizeof(status),fcmd) == NULL){
            usleep(10*1000);
            continue;
        }
        if((t=const_cast<char*>(strrchr(status,'\n')))==NULL){
            PLOG_ERROR(WIFI_TAG,"%s %d Line Over Flow or reach EOF\n", __FILE__, __LINE__);
            continue;
        }else if (NULL != (t = strstr(status, EXEC_SUCC))) {
            rval = 0;
            break;
        }else if (NULL != (t = strstr(status, EXEC_FAIL))) {
            rval = -1;
            break;
        }
    }
    fclose(fcmd);   
    mRunScripting = false;
    return rval;
}

void  WPAWiFiOps::handleCmd(shared_ptr<Ops> cmd)
{
    switch (cmd->ops)
    {
    case WIFI_OPS_ADD:
        if(cmd->mode==WIFI_MODE_STA){
            mSsid = cmd->ssid;
            mKey = cmd->key;
        }else  if(cmd->mode==WIFI_MODE_AP){
            configAP(cmd->ssid,cmd->key);
        }
        break;
    case WIFI_OPS_SWITCH:
        if(cmd->mode==WIFI_MODE_AP){
            setAPMode();
        }else if(cmd->mode==WIFI_MODE_STA){   
            if (!cmd->ssid.empty()){
                setSTAMode(cmd->ssid, cmd->key);
            }else{
                setSTAMode();
            }
        }
        break;
    default:
        break;
    }
}
void WPAWiFiOps::convertSSIDKey(string& out,const string& in)
{
    string newQuote="'\"'\"'";
    
    out=in;
    for(string::size_type pos=0;;pos+=newQuote.length())
    {
        if((pos=out.find('\'',pos))!=string::npos){
            out.replace(pos,1,newQuote);
        }else{
            break;
        }
    }
}
int WPAWiFiOps::setSTAMode()
{
    int ret;
    char cmdBuff[256];    

    mStaReady=false;
    mWiFiMode=WIFI_MODE_STA;

    string ssid,key;
    if (!mSsid.empty()){
        convertSSIDKey(ssid,mSsid);
        convertSSIDKey(key,mKey);
    }else{
        string strSsid, strKey;
        getSsidKey(strSsid, strKey);
        convertSSIDKey(ssid, strSsid);
        convertSSIDKey(key, strKey);
    }
    if (mLastestMode != WIFI_MODE_STA){
        return -1;
    }
    mListener->onEvent(WIFI_MODE_STA,WIFI_EVENT_CONNECTING);
    return setSTAMode(ssid,key);
}


bool WPAWiFiOps::getSsidKey(string& ssid, string& key)
{
    vector<string> vWifis;    
    mConfig->getWiFis(vWifis);

    if (vWifis.size()<2){
        return false;
    }

    ssid = vWifis[0];
    key = vWifis[1];   
    if (vWifis.size()==2){
        return true;
    } 

    if (vWifis.size()%2 != 0){
        return false;
    }
  
  for (int retry = 0; retry<1; retry++){
    vector<WiFiInFo> wifis;
    if (!scanWiFilist(wifis)){
        return false;
    }
    for(auto& wifi:wifis){
        for (int i=0; i<static_cast<int>(vWifis.size()); i+=2){
             if (wifi.ssid == vWifis[i]){
                 ssid = vWifis[i];
                 key = vWifis[i+1];    
                 return true;
             }
        }
        usleep(20*1000);
    }   
  }
  
    return false;        
}

int WPAWiFiOps::setSTAMode(string& ssid,string& key)
{
    int ret;
    char cmdBuff[256];

    mStaReady=false;
    mWiFiMode=WIFI_MODE_STA;

    mListener->onEvent(WIFI_MODE_STA,WIFI_EVENT_CONNECTING);

    //fix switch wifi bug
    mSsid = ssid;
    mKey = key;

    snprintf(cmdBuff,sizeof(cmdBuff),CMD_PREFIX"/usr/local/bin/wifi_start_sta.sh \'\"%s\"\' \'\"%s\"\'",ssid.c_str(),key.c_str());

    if ((ret = runScript(cmdBuff)) !=0){
        PLOG_ERROR(WIFI_TAG,"Set STA Mode Fail\n");
    }else{
        mRunAnaly = true;
    }
    mListener->onEvent(WIFI_MODE_STA,ret!=0?WIFI_EVENT_STOP:WIFI_EVENT_CONNECTED);
    if(ret==0){
        mStaReady=true;
    }

    return ret;
}

int WPAWiFiOps::setAPMode()
{
    mStaReady=false;
    mWiFiMode=WIFI_MODE_AP;
    mListener->onEvent(WIFI_MODE_AP,WIFI_EVENT_CONNECTING);
    int ret=runScript(CMD_PREFIX"/usr/local/bin/wifi_start_ap.sh");
    if(ret!=0){
        PLOG_ERROR(WIFI_TAG,"Set AP Mode Fail\n");
    }
    mListener->onEvent(WIFI_MODE_AP,ret!=0?WIFI_EVENT_STOP:WIFI_EVENT_CONNECTED);
    return ret;
}

int WPAWiFiOps::configAP(string& ssid,string& key)
{
    int ret;
    char cmdBuff[256];

    string s,k;
    convertSSIDKey(s,ssid);
    convertSSIDKey(k,key);

    snprintf(cmdBuff,sizeof(cmdBuff),CMD_PREFIX"/usr/local/bin/wifi_config_ap.sh \'%s\' \'%s\'",s.c_str(),k.c_str());
    if((ret=system(cmdBuff))!=0){
        PLOG_ERROR(WIFI_TAG,"Config AP Fail\n");
    }
    return ret;
}

void WPAWiFiOps::reconnectWifi()
{         
    mSsid.erase();
    mKey.erase();
    clearOps();
    switchMode(getMode());  
}


bool WPAWiFiOps::scanWiFilist(vector<WiFiInFo>& wifis )
{
    system(CMD_PREFIX"wl scan");

    int counter = 0;
    mScanning = true;
    while (mScanning && counter++<50){
        usleep(100*1000);
    }
    if (!mScanning){
        return mScanning;
    }

    string cmd = CMD_PREFIX"wl scanresults|grep -E \'SSID|RSSI\'  | grep -v 'BSSID'";

    FILE* scan = nullptr;
    scan=popen(cmd.c_str(),"r");
    if(scan==NULL){
        PLOG_ERROR(WIFI_TAG,"Open Scan WiFi list Fail\n");
        return mScanning;     
    }

    int err=0;    
    int count=0,mod;
    WiFiInFo wifi;
    char buff[256];
    while(fgets(buff,sizeof(buff),scan)){
        mod=count++%2;
        if(mod==0){
            string ssid=buff;
            static const string SSID="SSID: \"";
            auto idx=ssid.find(SSID);
            err=1;
            if(idx!=string::npos){
                wifi.ssid=ssid.substr(idx+SSID.size());
                if(wifi.ssid.size()>=3){
                    wifi.ssid.erase(wifi.ssid.end()-2,wifi.ssid.end());
                    unsigned char ssid[256] = {0};
                    mid_wifi_ssid_convert_utf8(ssid, wifi.ssid.c_str(), wifi.ssid.length());
                    wifi.ssid = (char*)ssid;
                    err=0;
                    PLOG_INFO(WIFI_TAG,"ssid:%s",ssid);
                }
            }
            if(err){
                PLOG_WARN(WIFI_TAG,"bad ssid:%s",buff);
            }
        }else{
            if(err){
                continue;
            }
            static const string RSSI="RSSI: ";
            static const string DBM=" dBm";
            string strRssi;
            strRssi.assign(buff);
            auto idx=strRssi.find(RSSI)+RSSI.size();
            auto dbmIdx=strRssi.find(DBM);
            if (idx != string::npos && dbmIdx!=string::npos){
                string value = strRssi.substr(idx, dbmIdx-idx);
                wifi.signal = atoi(value.c_str());
                wifis.push_back(wifi);
            }else{
                err=1;
                PLOG_WARN(WIFI_TAG,"bad rssi:%s",buff);
            }
        }
    }

    fclose(scan);
    sort(wifis.begin(),wifis.end(),CompGreater());

    return mScanning;
}

bool WIFI_has_config_ap()
{
    ifstream init(string(ROLLER_EYE_CONFIG_BASE)+"ap");
    return init.is_open();   
}
}
