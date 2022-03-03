#include<fstream>
#include <algorithm>
#include<stdlib.h>
#include<string.h>
#include<unistd.h>
#include<time.h>
#include<iostream>
#include"plt_assert.h"
#include<sys/fcntl.h>
#include<sys/stat.h>
#include<sstream>
#include"roller_eye/plt_config.h"
#include"roller_eye/plog_plus.h"
#include "roller_eye/version_info.h"

namespace roller_eye{
const char* CONFIG_TAG="config";
//
SingleLine::SingleLine(const string& name,bool neccerray):
Config(name,neccerray)
{
}
void SingleLine::load()
{
    ifstream file(mPath);
    if(!file.is_open()){
        if(mNeccessary){
            printf("config[%s] not exist\n",mPath.c_str());
            plt_assert(false);
        }else{
            printf("config[%s] not exist\n",mPath.c_str());
            return;
        }
    }
    file>>mCfg;
    plt_assert(!file.fail());
    printf("Loading config[%s:%s]\n",mName.c_str(),mCfg.c_str());
}
  void SingleLine::store()
  {
      ofstream file(mPath,ios::trunc);
      file<<mCfg;
      plt_assert(!file.fail());
  }
 //
AutoGenLine::AutoGenLine(const string& name,int len)
 :Config(name),
 mLen(len),
 mRegen(false)
 {

 }
 void AutoGenLine::setRegen(bool re)
 {
    mRegen=re;
    set("");
 }
void  AutoGenLine::load()
{
    static const char* map="0123456789qwertyuioplkjhgfdsazxcvbnmQWERTYUIOPLKJHGFDSAZXCVBNM";
    if(mCfg.length()==0)
    {
        int fd=open(mPath.c_str(),O_CREAT|O_RDWR,0600);

        plt_assert(fd>=0);
        plt_assert(lockf(fd,F_LOCK,0)==0);
         
        struct stat s;
        plt_assert(fstat(fd,&s)==0);
        if(mRegen||(s.st_size!=0&&s.st_size!=mLen)){
            if(ftruncate(fd,0)!=0){
                printf("Truncate  file[%s] fail\n",mPath.c_str());
                plt_assert(false);
            }
            s.st_size=0;
        }
        mCfg.resize(mLen);
        if(s.st_size==0){
            srand(plt_get_mono_time_ms());
            for(int i=0;i<mLen;i++)
            {
                mCfg[i]=map[rand()%62];
            }
            plt_assert(write(fd,mCfg.c_str(),mCfg.length())==mLen);
        }else{
            plt_assert(read(fd,const_cast<char*>(mCfg.data()),mLen)==mLen);
        }
        plt_assert(lseek(fd,0,SEEK_SET)==0);
        plt_assert(lockf(fd,F_ULOCK,0)==0);
        close(fd);
    }
    printf("Loading config[%s:%s]\n",mName.c_str(),mCfg.c_str());
}
void  AutoGenLine::store()
{

}
//
MultiLines::MultiLines(const string& name,bool neccerray):
  Config(name,neccerray)
  {
      
  }
void MultiLines::load()
{
    char buff[128];
    ifstream file(mPath);
    if(!file.is_open()){
        if(mNeccessary){
            printf("config[%s] not exist\n",mPath.c_str());
            plt_assert(false);
        }else{
            return;
        }
    }
    mCfg.clear();
    while(!file.eof()){
        file.getline(buff,sizeof(buff));
        if(strlen(buff)>0){
            mCfg.push_back(buff);
        }
    }
}
void MultiLines::store()
{
    ofstream file(mPath,ios::trunc);
    for(auto &v :mCfg){
        file<<v<<endl;
    }
    plt_assert(!file.fail());
}
//
JsonConfig::JsonConfig(const string& name,bool neccerray):
ProcessLockConfig(name,neccerray)
{
    printf("JsonConfig name: %s\n",name.c_str());
}
JsonConfig::JsonConfig(const string& path,const string& name,bool neccerray):
ProcessLockConfig(path, name,neccerray)
{
    printf("JsonConfig path:%s, name:%s\n",path.c_str(),name.c_str());
}
void JsonConfig::load()
{
    try{
        ifstream file(mPath);
        file>>mCfg;
    }catch(...){
        mCfg=nlohmann::json::object();
        printf("load json config[%s] fail\n",mPath.c_str());
    }
}
void JsonConfig::store()
{
    ofstream file(mPath,ios::trunc);
    file.width(1);
    file<<mCfg;
    plt_assert(!file.fail());
}

 //
PltConfig::PltConfig():
mSN("sn",false),
mVersion("version",false),
mLastPatrolName("lastPatrolName",false),
mWiFi("wifi",false),
mMotorSerial("motor_port"),
mMotorSpeed("motor_speed",false),
mMonitor(PARAM_SYSTEM_MONITOR_GROUP),
mSoundEffect(PARAM_SYSTEM_SOUND_EFFECT, false),
mMotionCfg(PARAM_SYSTEM_MOTION,false),
mProxyListCfg(ROLLER_EYE_SOCKPROXY_BASE, "proxy_list.json",true),
mMaxNavSize("nav_maxsize",false),
mMinBatForPowerOff("minbat_poweroff",false),
mMinBat("minbat",false),
mHwVersion("hw_ver")
{

}

PltConfig::~PltConfig()
{

}

string& PltConfig::getSN(string& val)
{
    mSN.reload();
    mSN.get(val);
    return val;
}

string& PltConfig::getHwVersion(string& val)
{
    mHwVersion.reload();
    mHwVersion.get(val);
    return val;
}

string& PltConfig::getLastPatrolName(string& val)
{
    mLastPatrolName.reload();
    mLastPatrolName.get(val);
    return val;
}

void  PltConfig::updateSN(const string& val)
{
    mSN.set(val);
}

void PltConfig::updateVersion(const string& val)
{    
    mVersion.set(val);
}

void PltConfig::updateLastPatrolName(const string& val)
{
    mLastPatrolName.set(val);
}

string& PltConfig::getMotorPort(string& val)
{
    mMotorSerial.get(val);
    plt_assert(val.length()>0);
    return val; 
}

vector<string>&  PltConfig::getWiFis(vector<string>& val)
{
    mWiFi.reload();
    mWiFi.get(val);
    return val;
}

   void PltConfig::getMaxNavSize(int& size)
   {
       string val;
       mMaxNavSize.get(val);
       if (!val.empty()){
            istringstream iss(val);  
            iss >> size;  
       }
   }

   void PltConfig::getMinBatForPowerOff(int& value)
   {
       string val;
       mMinBatForPowerOff.get(val);
       if (!val.empty()){
            istringstream iss(val);  
            iss >> value;  
       }
   }


    void PltConfig::getMinBat(int& value)
    {
      string val;
       mMinBat.get(val);
       if (!val.empty()){
            istringstream iss(val);  
            iss >> value;  
       }
    }

void PltConfig::updateWiFis(const vector<string> &val)
{
    mWiFi.set(val);
}
string& PltConfig::getMotorSpeed(string& val)
{
    mMotorSpeed.get(val);
    return val;
}
void PltConfig::setMotorSpeed(string& val)
{
    mMotorSpeed.set(val);
}
void PltConfig::getMonitorParam(nlohmann::json& val)
{
    mMonitor.reload();
    mMonitor.get(val);
}
void PltConfig::setMonitorParam(nlohmann::json& val)
{
    mMonitor.set(val);
}
void PltConfig::getMotionParam(nlohmann::json& val)
{
    mMotionCfg.reload();
    mMotionCfg.get(val);
}
void PltConfig::setMotionParam(nlohmann::json& val)
{
    mMotionCfg.set(val);
}

void PltConfig::getProxyList(nlohmann::json& val)
{    
    mProxyListCfg.reload();
    mProxyListCfg.get(val);
}
void PltConfig::getSoundEffectParam(nlohmann::json& val)
{
    mSoundEffect.reload();
    mSoundEffect.get(val);
}
void PltConfig::setSoundEffectParam(nlohmann::json& val)
{
    mSoundEffect.set(val);
}

ReadOnlyConfig::ReadOnlyConfig(string path)
{
    std::ifstream in(path);
    try{
        mConfig<<in;
    }catch(...){
        printf("invalid json config[%s]\n",path.c_str());
    }
}
ReadOnlyConfig::~ReadOnlyConfig()
{

}
     
int ReadOnlyConfig::getInt(string& key,int def)
{
    return get<int>(key,def);
}
bool ReadOnlyConfig::getBool(string& key,bool def)
{
    return get<bool>(key,def);
}
string ReadOnlyConfig::getString(string& key,string def)
{
    return get<string>(key,def);    
}
float ReadOnlyConfig::getFloat(string& key,float def)
{
    return get<float>(key,def);
}
template<typename T >
T ReadOnlyConfig::get(string& key,T def)
{
    T ret;
    try{
        ret=mConfig[key];
    }catch(...){
        ret=def;
    }
    std::stringstream ss;
    ss<<"config["<<key<<"]:"<<ret;
    printf("%s",ss.str().c_str());
    return ret;
}
DeviceDefaultConfig::DeviceDefaultConfig():
mCfg(ROLLER_EYE_CONFIG_BASE"device_default_config")
{

}
bool DeviceDefaultConfig::getMotorAdjust()
{
    string key="motor_adjust";
    return mCfg.getBool(key);
}
bool DeviceDefaultConfig::getMotorFixDir()
{
    string key="motor_fix_dir";
    return mCfg.getBool(key,false);
}
bool DeviceDefaultConfig::getBoradFlipped()
{
    string key="board_flip";
    return mCfg.getBool(key,false);
}
int DeviceDefaultConfig::getMotorSetDirDelay()
{
    string key="motor_change_dir_delay";
    return mCfg.getInt(key,100);
}
float DeviceDefaultConfig::getMotorFilterAccSigma()
{
    string key="filter_acc_sigma";
    return mCfg.getFloat(key,0.1);
}
float DeviceDefaultConfig::getMotorFilterUpdateSigma()
{
    string key="filter_update_sigma";
    return mCfg.getFloat(key,0.0);
}
    int DeviceDefaultConfig::getMotorStopDelay()
    {
        string key="motor_stop_delay";
        return (mCfg.getInt(key,600)*1000);
    }
    int DeviceDefaultConfig::getMotorPoweroffDelay()
    {
         string key="motor_poweroff_delay";
        return (mCfg.getInt(key,60000)*1000);
    }
    float DeviceDefaultConfig::getNightModeDuration()
    {
        string key="night_mode_duration";
        return mCfg.getFloat(key,0.1);
    }
    int DeviceDefaultConfig::getNightModeMinCnt()
    {
         string key="night_mode_min_cnt";
        return mCfg.getInt(key,20);
    }    
    int DeviceDefaultConfig::getNightModeDayCnt()
    {
         string key="night_mode_day_cnt";
        return mCfg.getInt(key,2);
    }    
    int DeviceDefaultConfig::getNightModeSamples()
    {
         string key="night_mode_sample";
        return mCfg.getInt(key,10);
    }
    int DeviceDefaultConfig::getNightModeDaySamples()
    {
         string key="night_mode_day_sample";
        return mCfg.getInt(key,2);
    }
    float DeviceDefaultConfig::getNightModeVth1()
    {
         string key="night_mode_vth1";
        return mCfg.getFloat(key,5);
    }
    float DeviceDefaultConfig::getNightModeVth2()
    {
         string key="night_mode_vth2";
        return mCfg.getFloat(key,12.5);
    }

    float DeviceDefaultConfig::getNightModeBand()
    {
         string key="night_mode_band";
        return mCfg.getFloat(key,7.5);
    }

    float DeviceDefaultConfig::getNightModeVth3()
    {
         string key="night_mode_vth3";
        return mCfg.getFloat(key,15);
    }
    float DeviceDefaultConfig::getNightModeVth4()
    {
         string key="night_mode_vth4";
        return mCfg.getFloat(key,25);
    }

    float DeviceDefaultConfig::getNightModeBand2()
    {
         string key="night_mode_band2";
        return mCfg.getFloat(key,12);
    }

    float DeviceDefaultConfig::getNightTofObstacleThreshold()
    {
        string key="night_tof_obstacle_threshold";
        return mCfg.getFloat(key,1.0);
    }

    float DeviceDefaultConfig::getNightIrAdjustThreshold()
    {
        string key="night_IR_adjust_threshold";
        return mCfg.getFloat(key,100);
    }

    int DeviceDefaultConfig::getCameraFormat()
    {
        string key="camera_format";
        return mCfg.getInt(key,0);
    }
    int DeviceDefaultConfig::getCameraFlipType()
    {
        string key="camera_flip_type";
        return mCfg.getInt(key,0);
    }
    float DeviceDefaultConfig::getObstacleMinDistance()
    {
        string key="obstacle_min_distance";
        return mCfg.getFloat(key,0.1);
    }

    float DeviceDefaultConfig::getSlowSpeedDistance()
    {
        string key="slow_speed_distance";
        return mCfg.getFloat(key,0.22);
    }

    float DeviceDefaultConfig::getObstacleSpeed()
    {
        string key="obstacle_speed";
        return mCfg.getFloat(key,0.1);
    }
    bool DeviceDefaultConfig::getNavPubIMUPose()
    {
        string key="nav_pub_imu_pose";
        return mCfg.getBool(key,false);
    }
    float DeviceDefaultConfig::getNavIMUFilterGain()
    {
        string key="nav_imu_filter_gain";
        return mCfg.getFloat(key,0.1);
    }
    float DeviceDefaultConfig::getNavIMUFilterZeta()
    {
        string key="nav_imu_filter_zeta";
        return mCfg.getFloat(key,0.0);
    }
    float DeviceDefaultConfig::getMotionMinArea()
    {
        string key="motion_min_area";
        return mCfg.getFloat(key,1.0e-4);
    }
    float DeviceDefaultConfig::getMotionMaxArea()
    {
        string key="motion_max_area";
        return mCfg.getFloat(key,0.25);
    }
    float DeviceDefaultConfig::getTrackLookDistance()
    {
        string key="track_look_distance";
        return mCfg.getFloat(key,0.2);
    }    
}
