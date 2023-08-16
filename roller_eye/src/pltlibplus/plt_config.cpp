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
            //PLOG_ERROR(CONFIG_TAG,"config[%s] not exist\n",mPath.c_str());
            plt_assert(false);
        }else{
            printf("config[%s] not exist\n",mPath.c_str());
            //PLOG_ERROR(CONFIG_TAG,"config[%s] not exist\n",mPath.c_str());
            return;
        }
    }
    file>>mCfg;
    plt_assert(!file.fail());
    //PLOG_DEBUG(CONFIG_TAG,"Loading config[%s:%s]\n",mName.c_str(),mCfg.c_str());
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
                //PLOG_ERROR(CONFIG_TAG,"Truncate  file[%s] fail\n",mPath.c_str());
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
    PLOG_DEBUG(CONFIG_TAG,"Loading config[%s:%s]\n",mName.c_str(),mCfg.c_str());
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
            //PLOG_ERROR(CONFIG_TAG,"config[%s] not exist\n",mPath.c_str());
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
    //PLOG_ERROR(CONFIG_TAG,"JsonConfig name: %s\n",name.c_str());
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
mLions("lionPars",false),
mMotorSerial("motor_port"),
mIbeaconInfo("ibeacon_info"),
mMotorSpeed("motor_speed",false),
mMonitor(PARAM_SYSTEM_MONITOR_GROUP),
mMotionCfg(PARAM_SYSTEM_MOTION,false),
mProxyListCfg(ROLLER_EYE_SOCKPROXY_BASE, "proxy_list.json",true),
mSoundEffect(PARAM_SYSTEM_SOUND_EFFECT, false),
mMaxNavSize("nav_maxsize",false),
mMinBatForPowerOff("minbat_poweroff",false),
mMinBat("minbat",false)
{

}

PltConfig::~PltConfig()
{

}
string& PltConfig::getSN(string& val)
{
    mSN.get(val);
    return val;
}

string& PltConfig::getHwVersion(string& val)
{
    val = HW_VERSION;
    //plt_assert(val.length()==strlen(HW_VERSION));
    return val;
}

bool PltConfig::isSTMotor()
{
    string hwVer;
    getHwVersion(hwVer);
    if (hwVer.length() ==10 &&
         ((hwVer[5] == '0' && hwVer[6] == '0')||
          (hwVer[5] == '0' && hwVer[6] == '2')) ){
              return true;
    }

    return false;
}

bool PltConfig::isChinaMotor()
{
    string hwVer;
    getHwVersion(hwVer);
    if (hwVer.length() ==10 &&
          ((hwVer[5] == '0' && hwVer[6] == '1')||
          (hwVer[5] == '0' && hwVer[6] == '3')||
          (hwVer[5] == '0' && hwVer[6] == '4')) ){
              return true;
    }

    return false;
}

bool PltConfig::isTrackModel()
{
	DeviceDefaultConfig cfg;
	nlohmann::json data;
	getMotionParam(data);

	if((1 ==  cfg.getTrackType()) || (1 == data["track"]))
	{
		return true;   /// is Track model ...
	}

    return false;
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

void PltConfig::getLionPars(vector<float>& vPars)
{
    vector<string> vStrPars;
    mLions.reload();
    mLions.get(vStrPars);

    for (auto it:vStrPars){
        istringstream iss(it);
        float num;
        iss >> num;
        vPars.push_back(num);
    }

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
vector<string>& PltConfig::getIbeaconInfo(vector<string>& val)
{
    mIbeaconInfo.get(val);
    plt_assert(val.size()==3);
    return val;
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
        //PLOG_INFO(CONFIG_TAG,"invalid json config[%s]\n",path.c_str());
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
double ReadOnlyConfig::getDouble(string& key,double def)
{
    return get<double>(key,def);
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

    float DeviceDefaultConfig::getBackupVx()
    {
        string key="backup_vx";
        return mCfg.getFloat(key,0.05);
    }

    float DeviceDefaultConfig::getBackupVy()
    {
        string key="backup_vy";
        return mCfg.getFloat(key,-0.14);
    }

    float DeviceDefaultConfig::getBackupWz()
    {
        string key="backup_wz";
        return mCfg.getFloat(key,0.0);
    }

    int DeviceDefaultConfig::getBackupDuration()
    {
        string key="backup_duration";
        return mCfg.getInt(key,1000);
    }

    int DeviceDefaultConfig::getTryCount()
    {
        string key="try_count";
        return mCfg.getInt(key,12);
    }


    float DeviceDefaultConfig::getAlignXOffset()
    {
        string key="align_x_offset";
        return mCfg.getFloat(key,0.0);   ///<<< decrease from 0.012
    }

    float DeviceDefaultConfig::getAlignMinX()
    {
        string key="align_min_x";
        return mCfg.getFloat(key,0.012);   ///<<< decrease from 0.012
    }
    float DeviceDefaultConfig::getAlignMinZ()
    {
        string key="align_min_z";
        return mCfg.getFloat(key,0.04);
    }
    float DeviceDefaultConfig::getAlignMinAngle()
    {
        string key="align_min_angle";
        return mCfg.getFloat(key,0.036);
    }
	int  DeviceDefaultConfig::getImuFilterOffsetCycleCount()
    {
        string key="filter_offset_cycle_count";
        return mCfg.getInt(key,64);
    }
	double DeviceDefaultConfig::getImuFilterOffsetThreshold()
    {
        string key="filter_offset_threshold";
        return mCfg.getDouble(key,0.10);
    }
	int   DeviceDefaultConfig::getImuFilterLogCycleCount()
    {
        string key="filter_log_cycle_count";
        return mCfg.getInt(key,5);
    }
	int   DeviceDefaultConfig::getTrackType()
    {
        string key="plt_track_type";
        return mCfg.getInt(key,0);
    }
	float DeviceDefaultConfig::getBackupRollSpeed()
    {
        string key="backup_roll_speed";
        return mCfg.getDouble(key,3.0);
    }
    float DeviceDefaultConfig::getTrackTypeBackupVx()
    {
        string key="track_type_backup_vx";
        return mCfg.getDouble(key, 0.05);
    }
    float DeviceDefaultConfig::getTrackTypeBackupVy()
    {
        string key="track_type_backup_vy";
        return mCfg.getDouble(key, -0.1);
    }
    float DeviceDefaultConfig::getTrackTypeBackupWz()
    {
        string key="track_type_backup_wz";
        return mCfg.getDouble(key, -2.5);
    }
    float DeviceDefaultConfig::getTrackTypeSwingRatio()
    {
        string key="track_type_swing_ratio";
        return mCfg.getDouble(key, -2.5);
    }
    int DeviceDefaultConfig::getTrackTypeTryCount()
    {
        string key="track_type_backup_try_count";
        return mCfg.getInt(key, 15);
    }
	float DeviceDefaultConfig::getTrackTypeBackupRollMinSpeed()
    {
        string key="track_type_backup_roll_min_speed";
        return mCfg.getDouble(key,2.5);
    }
	float DeviceDefaultConfig::getTrackTypeBackupRollSpeed()
    {
        string key="track_type_backup_roll_speed";
        return mCfg.getDouble(key,3.0);
    }
	float DeviceDefaultConfig::getTrackTypeBackupRollAngle()
    {
        string key="track_type_backup_roll_angle";
        return mCfg.getDouble(key,175.0);
    }
	float DeviceDefaultConfig::getBackupMoveDist()
    {
        string key="backup_move_dist";
        return mCfg.getFloat(key,0.05);
    }
	float DeviceDefaultConfig::getBackupMaxRollSpeed()
    {
        string key="backup_max_roll_speed";
        return mCfg.getFloat(key,1.34);
    }
	float DeviceDefaultConfig::getBackupMaxShiftSpeed()
    {
        string key="backup_max_shift_speed";
        return mCfg.getFloat(key,0.062);
    }
    float DeviceDefaultConfig::getAngleRatio()
    {
        string key="angle_ratio";
        return mCfg.getFloat(key, 2.5);
    }
    float DeviceDefaultConfig::getTrackTypeMoveForwardRatio()
    {
        string key="track_type_move_forward_ratio";
        return mCfg.getFloat(key, 1.3);
    }
    float DeviceDefaultConfig::getTrackTypeMoveBackwardRatio()
    {
        string key="track_type_move_backward_ratio";
        return mCfg.getFloat(key, 2.5);
    }
    float DeviceDefaultConfig::getTrackTypeGyroxClimbingThres()
    {
        string key="track_type_gyrox_climbing_thres";
        return mCfg.getFloat(key, 1.2);
    }
    float DeviceDefaultConfig::getTrackTypeFloorRollFactor()
    {
        string key="track_type_floor_roll_factor";
        return mCfg.getFloat(key, 2.0);
    }

        // For tuning
    float DeviceDefaultConfig::getRollExKp()
    {
        string key="rollEx_kp";
        return mCfg.getFloat(key, 1.0);
    }
    float DeviceDefaultConfig::getRollExKi()
    {
        string key="rollEx_ki";
        return mCfg.getFloat(key, 1.0);
    }
    float DeviceDefaultConfig::getRollExKd()
    {
        string key="rollEx_kd";
        return mCfg.getFloat(key, 1.0);
    }
    float DeviceDefaultConfig::getRollExMaxInitialSpeed()
    {
        string key="rollEx_max_initial_speed";
        return mCfg.getFloat(key, 3.5);
    }
}
