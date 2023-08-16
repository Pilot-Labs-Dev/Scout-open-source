#ifndef __ROLLER_EYE_CONFIGS_H__
#define __ROLLER_EYE_CONFIGS_H__
#include<string>
#include<mutex>
#include<vector>
#include"roller_eye/single_class.h"
#include"roller_eye/system_define.h"
#include"roller_eye/plt_tools.h"
#include"roller_eye/plog_plus.h"
#include"nlohmann/json.hpp"

using namespace std;
namespace roller_eye{
template <class T>
class Config{
public:
    Config(const string& name,bool neccerray=true):
    mReload(true),
    mName(name),
    mNeccessary(neccerray)
    {
        mPath=ROLLER_EYE_CONFIG_BASE+name;
    }
    Config(const string& path,const string& name,bool neccerray=true):
    mReload(true),
    mName(name),
    mNeccessary(neccerray)
    {
        mPath=path+name;
    }
    virtual ~Config()
    {

    }
    const string& getConfigPath()
    {
        return mPath;
    }
    virtual void get(T& t)
    {
         lock_guard<mutex> lock(mMutex);
         if(mReload)
        {
            load();
            mReload=false;
        }
        t=mCfg;
    }
    virtual void set(const T& t)
    {
         lock_guard<mutex> lock(mMutex);
         mCfg=t;
         store();
         mReload=true;
    }

    void reload()
    {
        lock_guard<mutex> lock(mMutex);
        mReload=true;
    }

protected:
    virtual void load()=0;
    virtual void store()=0;
    bool mReload;
    string mPath;
    string mName;
    mutex mMutex;
    T mCfg;
    bool mNeccessary;
};
class SingleLine:public Config<string>{
    public:
        SingleLine(const string& name,bool neccerray=true);
    private:
        void load();
        void store();
    };
 class MultiLines:public Config<vector<string>>{
    public:
        MultiLines(const string& name,bool neccerray=true);
    private:
        void load();
        void store();
    };
class AutoGenLine:public Config<string>{
    public:
        AutoGenLine(const string& name,int len);
        void setRegen(bool re);
    private:
        void load();
        void store();
        int mLen;
        bool mRegen;
    };
template <class T>
class ProcessLockConfig:public Config<T>{
public:
    ProcessLockConfig(const string& name,bool neccerray=true):
    Config<T>(name,neccerray),
    mLockFD(-1)
    {
    }
    ProcessLockConfig(const string& path, const string& name,bool neccerray=true):
    Config<T>(path,name,neccerray),
    mLockFD(-1)
    {
    }
    virtual void get(T& t)
    {
        if(lock()<0){
            return;
        }
        Config<T>::get(t);
        unlock();
    }
    virtual void set(const T& t)
    {
        if(lock()<0){
            return;
        }
        Config<T>::set(t);
        unlock();
    }
private:
    int lock()
    {
        mLockFD=open_lock_whole_file(Config<T>::mPath.c_str());
        if(mLockFD<0){
            PLOG_ERROR("config","open lock file [%s] fail\n",Config<T>::mPath.c_str());
            return -1;
        }
        return 0;
    }
    void unlock()
    {
        if(close_unlock_whole_file(mLockFD)<0){
            PLOG_ERROR("config","close unlock file [%s] fail\n",Config<T>::mPath.c_str());
        }
    }
    int mLockFD;
};
class JsonConfig:public ProcessLockConfig<nlohmann::json>{
    public:
        JsonConfig(const string& name,bool neccerray=false);
        JsonConfig(const string& path, const string& name,bool neccerray=false);
    private:
        void load();
        void store();
};
 class PltConfig:public SingleClass<PltConfig>{
     friend class SingleClass<PltConfig>;
public:

    string& getSN(string& val);
    string& getLastPatrolName(string& val);
    string& getHwVersion(string& val);

    string& getMotorPort(string& val);
    void updateLastPatrolName(const string& val);
    void updateSN(const string& val);
    void updateVersion(const string& val);

    vector<string>&  getWiFis(vector<string>& val);
    void updateWiFis(const vector<string> &val);

    vector<string>& getIbeaconInfo(vector<string>& val);

    string& getMotorSpeed(string& val);
    void setMotorSpeed(string& val);

    void getMonitorParam(nlohmann::json& val);
    void setMonitorParam(nlohmann::json& val);

    void getProxyList(nlohmann::json& val);

    void getLionPars(vector<float>& vPars);
    void getMaxNavSize(int& size);
    void getMinBatForPowerOff(int& value);
    void getMinBat(int& value);
    void getSoundEffectParam(nlohmann::json& val);
    void setSoundEffectParam(nlohmann::json& val);

    void getMotionParam(nlohmann::json& val);
    void setMotionParam(nlohmann::json& val);

    bool isSTMotor();
    bool isChinaMotor();
	bool isTrackModel();

protected:
    PltConfig();
     ~PltConfig();

    SingleLine mSN;
    SingleLine mVersion;
    SingleLine mLastPatrolName;
    MultiLines mWiFi;
    MultiLines mLions;
    SingleLine mMotorSerial;
    MultiLines mIbeaconInfo;
    SingleLine mMotorSpeed;
    JsonConfig mMonitor;
    JsonConfig mSoundEffect;
    JsonConfig mMotionCfg;
    JsonConfig mProxyListCfg;
    SingleLine mMaxNavSize;
    SingleLine mMinBatForPowerOff;
    SingleLine mMinBat;
 };
 class ReadOnlyConfig{
public:
     ReadOnlyConfig(string path);
     ~ReadOnlyConfig();

    int getInt(string& key,int def=0);
    bool getBool(string& key,bool def=true);
    string getString(string& key,string def="");
    float getFloat(string& key,float def=0.0f);
    double getDouble(string& key,double def=0.0);
    template<typename T >
    T get(string& key,T def);
private:
    nlohmann::json mConfig;
 };
 class DeviceDefaultConfig{
public:
    DeviceDefaultConfig();
    bool getMotorAdjust();
    bool getMotorFixDir();
    bool getBoradFlipped();
    int getMotorSetDirDelay();
    float getMotorFilterAccSigma();
    float getMotorFilterUpdateSigma();
    int getMotorStopDelay();
    int getMotorPoweroffDelay();
    float getNightModeDuration();
    int getNightModeDayCnt();
    int getNightModeMinCnt();
    int getNightModeSamples();
    int getNightModeDaySamples();
    float getNightModeVth1();
    float getNightModeVth2();
    float getNightModeBand();
    float getNightModeVth3();
    float getNightModeVth4();
    float getNightModeBand2();
    float getNightTofObstacleThreshold();
    float getNightIrAdjustThreshold();
    int getCameraFormat();
    int getCameraFlipType();
    float getObstacleMinDistance();
    float getSlowSpeedDistance();
    float getObstacleSpeed();
    bool getNavPubIMUPose();
    float getNavIMUFilterGain();
    float getNavIMUFilterZeta();
    float getMotionMinArea();
    float getMotionMaxArea();
    float getTrackLookDistance();
    float getBackupVx();
    float getBackupVy();
    float getBackupWz();
    int   getBackupDuration();
    int   getTryCount();
    float getAlignXOffset();
    float getAlignMinX();
    float getAlignMinZ();
    float getAlignMinAngle();
	int   getImuFilterOffsetCycleCount();
	double getImuFilterOffsetThreshold();
	int   getImuFilterLogCycleCount();
	float getBackupRollSpeed();
	float getBackupMoveDist();
    float getAngleRatio();
	float getBackupMaxRollSpeed();
    float getBackupMaxShiftSpeed();
	int   getTrackType();

    float getTrackTypeBackupVx();
    float getTrackTypeBackupVy();
    float getTrackTypeBackupWz();
    float getTrackTypeSwingRatio();
    int getTrackTypeTryCount();
	float getTrackTypeBackupRollMinSpeed();
	float getTrackTypeBackupRollSpeed();
	float getTrackTypeBackupRollAngle();
    float getTrackTypeMoveForwardRatio();
    float getTrackTypeMoveBackwardRatio();
    float getTrackTypeGyroxClimbingThres();
    float getTrackTypeFloorRollFactor();
    float getRollExKp();
    float getRollExKi();
    float getRollExKd();
    float getRollExMaxInitialSpeed();
    // float getTuneNumber1();
    // float getTuneNumber2();
private:
    ReadOnlyConfig mCfg;
 };


  class proxyListConfig{
public:
    proxyListConfig();
    string getString(string& key,string def="");
private:
    ReadOnlyConfig mCfg;
 };

}
#endif