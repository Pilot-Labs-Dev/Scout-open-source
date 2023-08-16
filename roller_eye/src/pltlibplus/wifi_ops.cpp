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
        resetIsSwitch();
        handleCmd(cmd);
    }
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
    PLOG_INFO(APP_NODE_TAG,"switchMode %d\n",mode);
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
    PLOG_INFO(APP_NODE_TAG,"switchMode %d %s %s\n",mode, ssid.c_str(), key.c_str());
    return 0;
}

void WiFiOps::setListener(Eventlistener* lisnter)
{
    mListener=lisnter;
}
WiFiOps::WiFiOps():
mListener(nullptr),
mOpsQueue(MAX_WIFI_CMD_SIZE),
mRuning(false),
mWiFiMode(WIFI_MODE_UNKOWN),
mStatus(WIFI_EVENT_STOP)
{
    clearOps();
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

void WPAWiFiOps::updateWiFis(bool bUpdate)
{
    if (mSsid.empty()){
        return;
    }

    string ssid = mSsid;
    string key = mKey;
    // ROS_INFO("WiFi updateWiFis %s %s %s:%d %s\n",ssid.c_str(), key.c_str(), __FILE__ ,__LINE__,__FUNCTION__);

    if(!bUpdate){
        mSsid.erase();
        mKey.erase();
        return;
    }

    bool bExist = false;
    if (!mSsid.empty()){
        for (int i=0; i<mWiFis.size(); i+=2){
            if (mSsid == mWiFis[i]){
                bExist = true;
                break;
            }
        }
    }

    if (!bExist){
        mWiFis.push_back(ssid);
        mWiFis.push_back(key);
        // ROS_INFO("WiFi updateWiFis %s %s %s:%d %s\n",ssid.c_str(), key.c_str(), __FILE__ ,__LINE__,__FUNCTION__);
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
            PLOG_ERROR(WIFI_TAG,"Line Over Flow or reach EOF\n");
            continue;
        }else if (NULL != strstr(status, "Access Point: Not-Associated")) {
            PLOG_INFO(WIFI_TAG,"isDisconnect: %s\n", status);
            pclose(fcmd);
            return true;
        }else if (NULL != strstr(status ,"Access Point")){
            //PLOG_INFO(WIFI_TAG,"isDisconnect: %s\n", status);
        }else if (NULL != strstr(status ,"Signal level")){
            //PLOG_INFO(WIFI_TAG,"isDisconnect: %s\n", status);
        }
    }
    pclose(fcmd);
    return false;
}

void WPAWiFiOps::runLoop()
{
    char status[512]={0};

    system(CMD_PREFIX"killall wpa_cli");
    sleep(3);

    PLOG_INFO(WIFI_TAG,"WiFi runLoop switchMode \n");
    switchMode(mWiFis.empty()?WIFI_MODE_AP:WIFI_MODE_STA);
   PLOG_INFO(WIFI_TAG,"quit WPAWiFiOps::runLoop() ");
}

void WPAWiFiOps::stopScript(int kill)
{
        PLOG_INFO(WIFI_TAG,"%d\n",  kill);
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

void WPAWiFiOps::updateScanResult(vector<WiFiInFo> vWifi)
{
    lock_guard<mutex> lock(mMutex);
    m_qvWifis.push(vWifi);
    if (m_qvWifis.size() > 6){
        m_qvWifis.pop();
    }
}


string WPAWiFiOps::getBssid(const string& ssid)
{
    int dbm = -200;
    string bssid;
    lock_guard<mutex> lock(mMutex);
    bool is11g = false;
    for (auto i=0; i<m_qvWifis.size(); i++){
        vector<WiFiInFo> wifis = m_qvWifis.front();
        m_qvWifis.pop();
        m_qvWifis.push(wifis);
        for (auto wifi:wifis){
            if (ssid == wifi.ssid){

                PLOG_INFO(WIFI_TAG,"ssid: %s, freq: %f, bssid: %s \n", ssid.c_str(), wifi.bssid.c_str(), wifi.freq);
                if (is11g && wifi.freq>=5){
                    continue;
                }

                if ((!is11g &&wifi.freq<5) ||
                        wifi.signal > dbm){
                    PLOG_INFO(WIFI_TAG,"ssid: %s, freq: %f, bssid: %s  signal: %d\n", ssid.c_str(), wifi.bssid.c_str(), wifi.freq, wifi.signal);
                    dbm = wifi.signal;
                    bssid = wifi.bssid;
                }

                if (wifi.freq<5){
                    is11g = true;
                }
            }
        }
    }

    return bssid;
}

int WPAWiFiOps::runScript(const string& cmd)
{
      char status[1024]={0};

//#define RETURNED_VALUE "__returned_value_"
#define EXEC_SUCC "start wifi ok"
#define EXEC_FAIL "start wifi fail"

    PLOG_INFO(WIFI_TAG,"%s\n",  cmd.c_str());
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
        PLOG_INFO(WIFI_TAG,"%s\n",  status);
    }
    PLOG_INFO(WIFI_TAG,"quit runScript");

    pclose(fcmd);
    mRunScripting = false;
    return rval;
}

void  WPAWiFiOps::handleCmd(shared_ptr<Ops> cmd)
{
    PLOG_DEBUG(WIFI_TAG,"recive cmd=%d,mode=%d,ssid=%s,key=%s\n",cmd->ops,cmd->mode,cmd->ssid.c_str(),cmd->key.c_str());
    switch (cmd->ops)
    {
    case WIFI_OPS_ADD:
        if(cmd->mode==WIFI_MODE_STA){
            mSsid = cmd->ssid;
            mKey = cmd->key;
        }else  if(cmd->mode==WIFI_MODE_AP){
            configAP(cmd->ssid,cmd->key);
            PLOG_INFO(WIFI_TAG,"WIFI_MODE_AP\n");
        }
        break;
    case WIFI_OPS_SWITCH:
        if(cmd->mode==WIFI_MODE_AP){
            setAPMode();
        }else if(cmd->mode==WIFI_MODE_STA){
            PLOG_INFO(WIFI_TAG,"WiFi setSTAMode\n");
            if (!cmd->ssid.empty()){
                //string bssid = getBssid(cmd->ssid);
                string bssid;
                getSsidKey(cmd->ssid, cmd->key, bssid);
                if (!isSwitch()){
                    setSTAMode(cmd->ssid, cmd->key, bssid);
                }
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

    PLOG_INFO(WIFI_TAG,"WiFi setSTAMode\n");
    mStaReady=false;
    mWiFiMode=WIFI_MODE_STA;
    if (mLastestMode != WIFI_MODE_STA){
        return -1;
    }

    if (WPAWiFiOps::getInstance()->getStatus() != WIFI_EVENT_CONNECTING){
        mListener->onEvent(WIFI_MODE_STA,WIFI_EVENT_CONNECTING);
    }
    string ssid,key, bssid;
    if (!mSsid.empty()){
        convertSSIDKey(ssid,mSsid);
        convertSSIDKey(key,mKey);
        //bssid = getBssid(ssid);
        getSsidKey(ssid, key, bssid);
    }else{
        string strSsid, strKey;
        getSsidKey(strSsid, strKey, bssid);
        convertSSIDKey(ssid, strSsid);
        convertSSIDKey(key, strKey);
    }

    PLOG_INFO(WIFI_TAG,"WiFi setSTAMode  %s %s %s \n",
                                                      ssid.c_str(), key.c_str(), bssid.c_str());
    if (isSwitch()){
        return -1;
    }
    return setSTAMode(ssid, key, bssid);
}


bool WPAWiFiOps::getSsidKey(string& ssid, string& key, string& bssid)
{
    vector<string> vWifis;

    if (ssid.empty()){
        mConfig->getWiFis(vWifis);
        if (vWifis.size()<2){
            return false;
        }
        ssid = vWifis[0];
        key = vWifis[1];
        PLOG_INFO(WIFI_TAG,"%s %s\n",ssid.c_str(),key.c_str());
    }else{
        vWifis.push_back(ssid);
        vWifis.push_back(key);
    }

    if (vWifis.size()%2 != 0){
        return false;
    }
    PLOG_INFO(WIFI_TAG,"%s\n",ssid.c_str());
    plt_system(CMD_PREFIX"/usr/local/bin/scan_wifi_init.sh");
    startScanWifi();
    vector<WiFiInFo> wifis;
    //usleep(5*1000*1000);
     for (int retry = 0; retry<3 && !isSwitch(); retry++){
        PLOG_INFO(WIFI_TAG,"retry: %d\n",retry);
        if (!scanWiFilist(wifis, retry>0)){
            PLOG_ERROR(WIFI_TAG,"scan failed");
            stopScanWifi();
            return false;
        }

        for(auto& wifi:wifis){
            for (int i=0; i<vWifis.size(); i+=2){
                if (wifi.ssid == vWifis[i]){
                    ssid = vWifis[i];
                    key = vWifis[i+1];
                    bssid = wifi.bssid;
                    PLOG_INFO(WIFI_TAG,"%s\n",ssid.c_str());
                    stopScanWifi();
                    return true;
                }
            }
        }

        for (int i=0; i<20 && !isSwitch(); i++){
            usleep(50*1000);
        }
    }

    stopScanWifi();
    PLOG_INFO(WIFI_TAG,"%s\n",ssid.c_str());

    return false;
}

int WPAWiFiOps::setSTAMode(string& ssid,string& key, string bssid)
{
    int ret;
    char cmdBuff[256];

    PLOG_ERROR(WIFI_TAG,"WiFi setSTAMode %s %s %s\n", ssid.c_str(), key.c_str(), bssid.c_str());
    mStaReady=false;
    mWiFiMode=WIFI_MODE_STA;

    if (WPAWiFiOps::getInstance()->getStatus() != WIFI_EVENT_CONNECTING){
        mListener->onEvent(WIFI_MODE_STA,WIFI_EVENT_CONNECTING);
    }
    mSsid = ssid;
    mKey = key;
    if (bssid.length()>10){
        snprintf(cmdBuff,sizeof(cmdBuff),CMD_PREFIX"/usr/local/bin/wifi_start_sta.sh \'\"%s\"\' \'\"%s\"\'  \'%s\'",
                          ssid.c_str(),key.c_str(), bssid.c_str());
    }else{
        snprintf(cmdBuff,sizeof(cmdBuff),CMD_PREFIX"/usr/local/bin/wifi_start_sta.sh \'\"%s\"\' \'\"%s\"\'",
                          ssid.c_str(),key.c_str());
    }
    if ((ret = runScript(cmdBuff)) !=0){
        PLOG_ERROR(WIFI_TAG,"Set STA Mode Fail\n");
    }else{
        mRunAnaly = true;
    }
    mListener->onEvent(WIFI_MODE_STA,ret!=0?WIFI_EVENT_STOP:WIFI_EVENT_CONNECTED);
    if(ret==0){
        mStaReady=true;
    }
    PLOG_INFO(WIFI_TAG,"WiFi setSTAMode return\n");
    return ret;
}

int WPAWiFiOps::setAPMode()
{
    PLOG_ERROR(WIFI_TAG,"WiFi setAPMode\n");
    mStaReady=false;
    mWiFiMode=WIFI_MODE_AP;
    if (WPAWiFiOps::getInstance()->getStatus() != WIFI_EVENT_CONNECTING){
        mListener->onEvent(WIFI_MODE_AP,WIFI_EVENT_CONNECTING);
    }
    //int ret=system(CMD_PREFIX"/usr/local/bin/wifi_start_ap.sh");
    int ret=runScript(CMD_PREFIX"/usr/local/bin/wifi_start_ap.sh");
    if(ret!=0)
    {
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
    if((ret=system(cmdBuff))!=0)
    {
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

bool WPAWiFiOps::scanWiFilist(vector<WiFiInFo>& wifis, bool bInit )
{
    if (!bInit){
        system(CMD_PREFIX"wl scan");
    }

    int counter = 0;
    mScanning = true;
    PLOG_INFO(WIFI_TAG,"waiting scan %d\n",isScanning());
    while (!bInit && isScanning() && counter++<50){
        usleep(100*1000);
    }
    if (!isScanning()){
        return isScanning();
    }

    PLOG_INFO(WIFI_TAG,"call wl scanresults %d %d\n",isScanning(),counter);
    string cmd = CMD_PREFIX"wl scanresults|grep -E \'SSID|RSSI\'";

    FILE* scan = nullptr;
    scan=popen(cmd.c_str(),"r");
    if(scan==NULL){
        PLOG_ERROR(WIFI_TAG,"Open Scan WiFi list Fail\n");
        return isScanning();
    }

    int err=0;
    int count=0,mod;
    WiFiInFo wifi;
    char buff[256];
    while(fgets(buff,sizeof(buff),scan) && isScanning()){
        mod=count++%3;
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
        }else if (1==mod){
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
            }else{
                err=1;
                PLOG_WARN(WIFI_TAG,"bad rssi:%s",buff);
            }
        }else{
            if(err){
                continue;
            }
            static const string BSSID="BSSID: ";
            string strBssid;
            strBssid.assign(buff);
            auto idx=strBssid.find(BSSID);
             if (idx != string::npos){
                 idx += BSSID.size();
             }
            if (idx != string::npos){
                static const string BLANK=" ";
                auto blankIdx=strBssid.find(BLANK, idx+1);
                string value = strBssid.substr(idx, blankIdx-idx);
                wifi.bssid = value;
                wifis.push_back(wifi);
                PLOG_INFO(WIFI_TAG,"bssid:%s",wifi.bssid.c_str());
            }else{
                err=1;
                PLOG_WARN(WIFI_TAG,"bad bssid:%s",buff);
            }
        }

    }

    pclose(scan);
    sort(wifis.begin(),wifis.end(),CompGreater());

    PLOG_WARN(WIFI_TAG,"wifis size:%d",wifis.size());
    return isScanning();
}

bool WIFI_has_config_ap()
{
    ifstream init(string(ROLLER_EYE_CONFIG_BASE)+"ap");
    return init.is_open();
}
}
