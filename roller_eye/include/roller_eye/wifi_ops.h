#ifndef __ROLLER_EYE_WIFI_OPS__H__
#define __ROLLER_EYE_WIFI_OPS__H__
#include <atomic>
#include <vector>
#include <mutex>
#include"roller_eye/single_class.h"
#include"roller_eye/block_queue.h"
#include"roller_eye/plt_config.h"

#define MAX_WIFI_CMD_SIZE           100
namespace roller_eye{
    struct WiFiInFo{
    string bssid;
    string ssid;
    int signal;
    int noisy;
    float quality;
    float freq;
    int channel;
};

class WiFiOps{
public:
enum{
    WIFI_EVENT_CONNECTED,
    WIFI_EVENT_DISCONNECT,
    WIFI_EVENT_CONNECTING,
    WIFI_EVENT_WRONG_KEY,
    WIFI_EVENT_CONN_FAIL,
    WIFI_EVENT_STOP,
};
enum{
    WIFI_MODE_STA,
    WIFI_MODE_AP,
    WIFI_MODE_UNKOWN
};

enum{
    WIFI_OPS_NONE=-1,
    WIFI_OPS_SWITCH,
    WIFI_OPS_ADD,
    WIFI_OPS_STOP
};
    class Eventlistener;

    WiFiOps();
    virtual ~WiFiOps();
    void run();
    void cmdProc();
    int addSSIDKey(int mode,string& ssid,string& key);
    int switchMode(int mode);
    int switchMode(int mode,string& ssid,string& key);
    void setListener(Eventlistener* lisnter);
    bool isSwitch(){return mIsSwitch;}
    void resetIsSwitch() {mIsSwitch = false;}
    void setIsSwitch() {mIsSwitch = true;}
    void clearOps();
    int getStatus() {return mStatus;}
    void setStatus(int status) {mStatus = status;}

protected:
    struct Ops{
        int ops;
        int mode;
        string ssid;
        string key;
    };

    virtual void runLoop()=0;
    virtual void handleCmd(shared_ptr<Ops> cmd)=0;
protected:
    Eventlistener *mListener;
    int mWiFiMode;
    Ops mOps;
private:
    BlockQueue<shared_ptr<Ops>> mOpsQueue;
    atomic_flag mRuning;
    mutex mMutex;
    int mStatus;
    bool mIsSwitch = false;
};
class WiFiOps::Eventlistener{
public:
    Eventlistener();
    virtual ~Eventlistener();
    virtual void onEvent(int mode,int status)=0;
};
class WPAWiFiOps:public WiFiOps,public SingleClass<WPAWiFiOps>{
    friend class SingleClass<WPAWiFiOps>;

    public:
    int  getMode(){return mWiFiMode;}
    void updateWiFis(bool bUpdate=true);

    void reconnectWifi();

    void startScanWifi() {mScanning = true;}
    bool scanWiFilist(vector<WiFiInFo>& wifis, bool bInit = false);
    void updateScanResult(vector<WiFiInFo> vWifi);
    void stopScanWifi(){mScanning=false;}

    bool isDisconnect();
    bool isScanning(){return mScanning;}
    bool isRunningScript(){return mRunScripting;}

    void setLastMode(int mode){mLastestMode = mode;}
    void stopScript(int kill);

private:
    //void loop();
    void runLoop();
    void handleCmd(shared_ptr<Ops> cmd);
    int runScript(const string& cmd);
    string getBssid(const string& ssid);
private:
    WPAWiFiOps();
    ~WPAWiFiOps();

    int setAPMode();
    int setSTAMode();
    int setSTAMode(string& ssid,string& key, string bssid="");
    bool getSsidKey(string& ssid, string& key, string& bssid);
    int configAP(string& ssid,string& key);
    void convertSSIDKey(string& out,const string& in);
    PltConfig *mConfig;
    vector<string> mWiFis;
    bool mStaReady = false;
    bool mScanning = false;

    string mSsid;
    string mKey;

    int mLastestMode = WIFI_MODE_UNKOWN;
    bool mRunScripting = false;
    bool mRunAnaly = false;
    mutex mMutex;
    queue<vector<WiFiInFo>> m_qvWifis;
};
bool WIFI_has_config_ap();

}
#endif