#ifndef ROLLER_EYE_UTIL_CLASS_H
#define ROLLER_EYE_UTIL_CLASS_H
#include<string>
#include<vector>
#include <sys/statfs.h>
#include <pthread.h>
#include <sched.h>

using namespace std;
namespace roller_eye
{
    class TimeCost{
        public:
            TimeCost();
            void reset();
            int cost();
        private:
            uint64_t mCurrent;
    };
    bool readVector3fCalibration(const string &file,double calib[3]);
    void saveVector3fCalibration(const string &file,double calib[3]);

    bool readCalibration(const string &file,double *calib,int cnt);

    void saveCalibration(const string &file,double *calib,int cnt);

    bool listDir(const string& path,vector<string>& files,bool excludeDir=true);
    
    bool detectProcessIsExited(string processName);

    void stopProgramming();

    bool getFreeSpace(string dirName, unsigned long long *free);

    string runCmdAndGetResult(string cmdStr);
    int api_get_thread_policy (pthread_attr_t *attr);
    void api_show_thread_priority (pthread_attr_t *attr,int policy);
    int api_get_thread_priority (pthread_attr_t *attr);
    int api_set_thread_priority (pthread_attr_t *attr, int priority);
    void api_set_thread_policy (pthread_attr_t *attr,int policy);
} // namespace roller_eye

#endif