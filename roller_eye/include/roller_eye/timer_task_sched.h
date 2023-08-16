#ifndef __ROLLER_EYE_TIMER_TASK_SCHED_H__
#define __ROLLER_EYE_TIMER_TASK_SCHED_H__
#include<string>
#include<vector>
#include<memory>
#include<ctime>
using namespace std;
namespace roller_eye{
enum{
    TIMER_TASK_REPEAT_ONCE,//0
    TIMER_TASK_REPEAT_DAY_CLOSE,//1
    TIMER_TASK_REPEAT_DAY,//2
    TIMER_TASK_REPEAT_WEEK,//3
    TIMER_TASK_REPEAT_MONTH//4
};
class TimerTaskScheduler;
class TimerTask{
public:
    int id;
    string name;
    string type;
    int repeatType;
    vector<int> days;
    struct tm startTime;
    int expire;   
    string param;
    bool notice;
    bool active;

    TimerTask();
    TimerTask(TimerTask&task);
    TimerTask&operator=(TimerTask&task);
private:
    friend class TimerTaskScheduler;
    void copy(TimerTask&task);
    bool schedued;
    struct tm scheduleTime;
 };

typedef shared_ptr<TimerTask> TimerTaskPtr;

 class TimerTaskScheduler{
public:
    TimerTaskScheduler();
    ~TimerTaskScheduler();
    
    int addTask(TimerTaskPtr task);
    int modTask(TimerTaskPtr task);
    int delTask(int id);
    int delTask(string& name);
    TimerTaskPtr findTask(int id);

    vector<TimerTaskPtr> getTasks(const string& type,int startId,int size);
    TimerTaskPtr getNextTask(int& delay);
    void schedTask(TimerTaskPtr task);
private:
    TimerTaskPtr loadTask(string& name);
    int saveTask(TimerTaskPtr task);
    int genID();
    int getDaySec(struct tm *t);
    bool findDay(TimerTaskPtr& task,int day);
    bool sameDay(struct tm *l,struct tm* r);

    vector<TimerTaskPtr> mTask;
    string mRootPath;
 };
}
#endif