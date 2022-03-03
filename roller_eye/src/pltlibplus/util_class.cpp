#include"roller_eye/util_class.h"
#include"plt_tools.h"
#include"plog_plus.h"
#include<libgen.h>
#include <sys/stat.h>
#include <sys/types.h>
#include<cstdio>
#include<fstream>
#include <dirent.h>
#include <sys/types.h>
#include <time.h>

namespace roller_eye
{
    static const std::string UTIL_CLASS_TAG="utils";

    TimeCost::TimeCost()
    {
        reset();
    }
    void TimeCost::reset()
    {
        mCurrent=plt_get_mono_time_ms();
    }
    int TimeCost::cost()
    {
        return (int)(plt_get_mono_time_ms()-mCurrent);
    }
    bool readVector3fCalibration(const string &file,double calib[3])
    {
        return readCalibration(file,calib,3);
    }
    void saveVector3fCalibration(const string &file,double calib[3])
    {
        saveCalibration(file,calib,3);
    }
    bool readCalibration(const string &file,double *calib,int cnt)
    {
        int i;
        ifstream cfg(file);

        for( i=0;i<cnt;i++){
            cfg>>calib[i];
            if(cfg.fail()){
                PLOG_ERROR(UTIL_CLASS_TAG,"read %s fail\n",file.c_str());
                break;
            }
        }
        if(i!=cnt){
            memset(calib,0,cnt*sizeof(double));
            return false;
        }

        PLOG_INFO(UTIL_CLASS_TAG,"read %s ok",file.c_str());
        return true;
    }
    void saveCalibration(const string &file,double *calib,int cnt)
    {
        char path[128];
        if(*file.rbegin()=='\\'){
            PLOG_ERROR(UTIL_CLASS_TAG,"input  %s is path\n",file.c_str());
            return;
        }
        snprintf(path,sizeof(path),"%s",file.c_str());

        char* dir=dirname(path);
        if(access(dir,F_OK)!=0){
            mkdir(dir,0755);
        }

        int i;
        ofstream cfg(file);

        PLOG_INFO(UTIL_CLASS_TAG,"save %s",file.c_str());

        for( i=0;i<cnt;i++){
            cfg<<calib[i]<<" ";
            if(cfg.fail()){
                PLOG_ERROR(UTIL_CLASS_TAG,"write  %s fail\n",file.c_str());
                break;
            }
        }
    }
    bool listDir(const string& path,vector<string>& files,bool excludeDir)
    {
        DIR* dir=opendir(path.c_str());
        if(dir==NULL){
            return false;
        }
        struct dirent *entry;
        while((entry=readdir(dir))!=NULL){
            if(strcmp(entry->d_name,".")==0||strcmp(entry->d_name,"..")==0){
                continue;
            }
            if(excludeDir&&entry->d_type==DT_DIR){
                continue;
            }
            files.push_back(entry->d_name);
        }
        closedir(dir);
        return true;   
    }

    bool detectProcessIsExited(string processName){
      bool isOk = false;
      FILE* fpRead = NULL;
      char re_str[64] = {0};
      char cmd[512] = {0};

      sprintf(cmd, "ps -ax | grep -v grep | grep -c '%s'", processName.c_str());

      fpRead = popen(cmd, "r");
      memset(re_str, '\0', sizeof(re_str));
      size_t readCnt = fread(re_str, sizeof(char), sizeof(re_str)-1, fpRead);

      pclose(fpRead);

      re_str[readCnt - 1] = '\0';

      if(atoi(re_str) >= 1){
        return true;
      }

      return false;
    }

    void stopProgramming(){
      if(detectProcessIsExited("python /userdata/roller_eye/scratch/scripts/")){
        system("ps -aux | grep -v grep | grep 'python /userdata/roller_eye/scratch/scripts/' | awk '{print $2}' | xargs kill -9");
      }
    }

    bool getFreeSpace(string dirName, unsigned long long *free){
      if(free == NULL || dirName.length() == 0)
        return false;

      struct statfs diskInfo;
      statfs(dirName.c_str(), &diskInfo);
      unsigned long long totalBlocks = diskInfo.f_bsize;
      unsigned long long freeDisk = diskInfo.f_bfree * totalBlocks;

      PLOG_DEBUG(APP_NODE_TAG, "%s freeSpace:%lldMB", dirName.c_str(), freeDisk / 1024 / 1024 );

      *free = freeDisk;
      return true;
    }

    string runCmdAndGetResult(string cmdStr){
      bool isOk = false;
      FILE *fpRead = NULL;
      char result[256] = {0};
      char cmd[512] = {0};

      sprintf(cmd, "%s", cmdStr.c_str());

      fpRead = popen(cmd, "r");
      memset(result, '\0', sizeof(result));
      size_t readCnt = fread(result, sizeof(char), sizeof(result) - 1, fpRead);

      pclose(fpRead);

      if(readCnt > 0){
        if(result[readCnt - 1] == '\n'){
          result[readCnt - 1] = '\0';
          readCnt = readCnt - 1;
        }
      }

      string resultStr(result, readCnt);

      return resultStr;
    }
  int api_get_thread_policy (pthread_attr_t *attr)
  {
    int policy;
    int rs = pthread_attr_getschedpolicy (attr, &policy);
    assert (rs == 0);

    switch (policy)
    {
        case SCHED_FIFO:
            printf ("policy = SCHED_FIFO\n");
            break;
        case SCHED_RR:
            printf ("policy = SCHED_RR");
            break;
        case SCHED_OTHER:
            printf ("policy = SCHED_OTHER\n");
            break;
        default:
            printf ("policy = UNKNOWN\n");
            break; 
    }
    return policy;
}

void api_show_thread_priority (pthread_attr_t *attr,int policy)
{
    int priority = sched_get_priority_max (policy);
    assert (priority != -1);
    printf ("max_priority = %d\n", priority);
    priority = sched_get_priority_min (policy);
    assert (priority != -1);
    printf ("min_priority = %d\n", priority);
}

int api_get_thread_priority (pthread_attr_t *attr)
{
    struct sched_param param;
    int rs = pthread_attr_getschedparam (attr, &param);
    assert (rs == 0);
    printf ("priority = %d\n", param.__sched_priority);
    return param.__sched_priority;
}

int api_set_thread_priority (pthread_attr_t *attr, int priority)
{
    struct sched_param param;
    int rs = pthread_attr_getschedparam (attr, &param);
    assert (rs == 0);
    param.__sched_priority = priority;
    rs = pthread_attr_setschedparam(attr, &param);
    assert (rs == 0);
    printf ("set priority = %d\n", param.__sched_priority);
    return rs;
}

void api_set_thread_policy (pthread_attr_t *attr,int policy)
{
    int rs = pthread_attr_setschedpolicy (attr, policy);
    assert (rs == 0);
    api_get_thread_policy (attr);
}
};
