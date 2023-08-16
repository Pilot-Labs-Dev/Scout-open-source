#include<fstream>
#include<cstdlib>
#include<algorithm>
#include"roller_eye/timer_task_sched.h"
#include"roller_eye/util_class.h"
#include"roller_eye/plt_tools.h"
#include"roller_eye/system_define.h"
#include"roller_eye/plt_assert.h"
#include"roller_eye/plog_plus.h"
#include"nlohmann/json.hpp"

#define TIMER_TASK_TAG          "TimerTask"
#define TASK_JSON_FIELD_ID      "id"
#define TASK_JSON_FIELD_NAME      "name"
#define TASK_JSON_FIELD_TYPE      "type"
#define TASK_JSON_FIELD_REPEAT_TYPE      "repeatType"
#define TASK_JSON_FIELD_DAY      "days"
#define TASK_JSON_FIELD_TIME                "time"
#define TASK_JSON_FIELD_EXPIRE      "expire"
#define TASK_JSON_FIELD_PARAM      "param"
#define TASK_JSON_FIELD_ACTIVE      "active"
#define TASK_JSON_FIELD_NOTICE      "notice"

#define TASK_JSON_TIME_FORMAT       "%Y-%m-%d %H:%M:%S"
#define TASK_MAX_ID                                     10000000
#define SECOND_PER_DAY                           (24*3600)

namespace roller_eye{
    TimerTask::TimerTask():
    schedued(false)
    {

    }
    TimerTask::TimerTask(TimerTask&task):
    schedued(false)
    {
        PLOG_DEBUG(TIMER_TASK_TAG,"construct copy");
        copy(task);
    }
    TimerTask&TimerTask::operator=(TimerTask&task)
    {
        PLOG_DEBUG(TIMER_TASK_TAG,"sign copy");
        if(&task!=this){
            copy(task);
        }else{
            PLOG_DEBUG(TIMER_TASK_TAG,"same instance");
        }
        return *this;
    }
    void TimerTask::copy(TimerTask&task)
    {
        id=task.id;
        name=task.name;
        type=task.type;
        repeatType=task.repeatType;
        days=task.days;
        startTime=task.startTime;
        expire=task.expire;   
        param=task.param;
        active=task.active;
    }
    TimerTaskScheduler::TimerTaskScheduler():
    mRootPath(TIMER_TASK_PATH)
    {
        srand(time(NULL));
        mk_depth_dir(mRootPath.c_str());
        vector<string> files;
        plt_assert(listDir(mRootPath,files));
        for(auto& file:files){
            auto task=loadTask(file);
            if(task){
                mTask.push_back(task);
            }else{
                PLOG_ERROR(TIMER_TASK_TAG,"load task[%s] fail",file.c_str());
            }
        }
    }
    TimerTaskScheduler::~TimerTaskScheduler()
    {

    }
    
    int TimerTaskScheduler::addTask(TimerTaskPtr task)
    {
        int ret;
        if(!task){
            return -1;
        }
        task->id=genID();
        task->schedued=false;
        if((ret=saveTask(task))==0){
            mTask.push_back(task);
        }
        return ret;
    }
    int TimerTaskScheduler::modTask(TimerTaskPtr task)
    {
        int ret;
        if(!task){
            return -1;
        }
        auto oldTask=findTask(task->id);
        if(!oldTask){
            PLOG_ERROR(TIMER_TASK_TAG,"task[%d] fail",task->id);
            return -1;
        }
        if((ret=saveTask(task))==0){
            *oldTask=*task;
            oldTask->schedued=false;
        }
        return ret;
    }
    int TimerTaskScheduler::delTask(int id)
    {
        int ret;
        for(auto it=mTask.begin();it!=mTask.end();++it){
            if((*it)->id==id){
                string path=mRootPath+to_string(id);
                if((ret=remove(path.c_str()))!=0){
                    PLOG_ERROR(TIMER_TASK_TAG,"delete file [%s] fail",path.c_str());
                }else{
                    mTask.erase(it);
                }
                return ret;
            }
        }
        return -1;
    }

    int TimerTaskScheduler::delTask(string& name)
    {
        int ret = -1;
        for(auto it=mTask.begin();it!=mTask.end();){
            if((*it)->name==name){
                string path=mRootPath+to_string((*it)->id);
                if((ret=remove(path.c_str()))!=0){
                    PLOG_ERROR(TIMER_TASK_TAG,"delete file [%s] fail",path.c_str());
                }
                it = mTask.erase(it);
                ret = 0;
            }else{
                it++;
            }
        }
        return ret;
    }
    TimerTaskPtr TimerTaskScheduler::findTask(int id)
    {
        for(auto it=mTask.begin();it!=mTask.end();++it){
            if((*it)->id==id){
                return *it;
            }
        }
        return nullptr;
    }
    vector<TimerTaskPtr> TimerTaskScheduler::getTasks(const string& type,int startId,int size)
    {
        vector<TimerTaskPtr> result;
        if(startId<0||size<=0){
            return result;
        }
        for(auto& task:mTask){
            if(startId==0){
                if(task->type==type){
                    result.push_back(task);
                    if(--size==0){
                        break;
                    }
                }
            }else{
                startId--;
            }
        }
        return result;
    }
    TimerTaskPtr TimerTaskScheduler::getNextTask(int& delay)
    {
        time_t n=time(NULL);
        struct tm t;
        localtime_r(&n,&t);

        int tt,tn,diff;
        TimerTaskPtr ptr=nullptr;

        tn=getDaySec(&t);
        delay=SECOND_PER_DAY-tn;

        PLOG_INFO(TIMER_TASK_TAG,"t.tm_year=%d", t.tm_year);
        for(auto& task:mTask){
            if(!task->active){
                PLOG_INFO(TIMER_TASK_TAG,"task[%d] inactive",task->id);
                continue;
            }
            tt=getDaySec(&task->startTime);
            diff=tt-tn;
            PLOG_INFO(TIMER_TASK_TAG,"[%d] task->startTime.tm_year=%d diff:%d delay:%d %d %d %d %d",
                                        task->id, task->startTime.tm_year, diff, delay, task->schedued, task->scheduleTime.tm_mday, t.tm_mday,task->repeatType);

            if(task->schedued&&task->repeatType==TIMER_TASK_REPEAT_ONCE){
                PLOG_WARN(TIMER_TASK_TAG,"bug found: repeat once,this task need to be deleted");
                continue;
            }
            if(task->schedued&&task->repeatType==TIMER_TASK_REPEAT_DAY_CLOSE){
                PLOG_WARN(TIMER_TASK_TAG,"bug found: repeat day close,this task need to be inactive");
                continue;
            }
            if(task->schedued&&task->scheduleTime.tm_mday!=t.tm_mday)//new day
            {
                task->schedued=false;
            }

             if(diff>0&&task->schedued){
                PLOG_WARN(TIMER_TASK_TAG,"bug found: feature task has scheduled");
                continue;               
            }
            if(task->schedued){
                continue;
            }
            switch(task->repeatType){
                case TIMER_TASK_REPEAT_ONCE:
                    if(!sameDay(&t,&task->startTime)){
                        PLOG_INFO(TIMER_TASK_TAG,"not same day");
                        continue;
                    }
                    break;
                case TIMER_TASK_REPEAT_DAY:
                case TIMER_TASK_REPEAT_DAY_CLOSE:
                    break;
                case TIMER_TASK_REPEAT_WEEK:
                    if(!findDay(task,t.tm_wday)){
                        continue;
                    }
                    break;
                case TIMER_TASK_REPEAT_MONTH:
                    if(!findDay(task,t.tm_mday)){
                        continue;
                    }
                    break;
                default:
                    PLOG_WARN(TIMER_TASK_TAG,"bug found ,unkonw repeat type");
                    continue;
            }

            if(task->expire>=0&&diff+task->expire<=0){//expire < 0,means never expire
                PLOG_INFO(TIMER_TASK_TAG,"task[%d] expire",task->id);
                continue;
            }
            if(diff<=delay){
                delay=diff;
                ptr=task;
            }
        }
        return ptr;
    }
    void TimerTaskScheduler::schedTask(TimerTaskPtr task)
    {
        if(!task){
            return;
        }
        task->schedued=true;
        time_t t=time(NULL);
        localtime_r(&t,&task->scheduleTime);
        if(task->repeatType==TIMER_TASK_REPEAT_ONCE){
            delTask(task->id);
        }
        if(task->repeatType==TIMER_TASK_REPEAT_DAY_CLOSE){
            task->active=false;
            if(saveTask(task)!=0){
                PLOG_ERROR(TIMER_TASK_TAG,"save inactive task[%d] fail",task->id);
            }
        }
    }
    TimerTaskPtr TimerTaskScheduler::loadTask(string& name)
    {
        string path=mRootPath+name;
        auto task=make_shared<TimerTask>();
        std::ifstream in(path);
        nlohmann::json data;
        try{
            data<<in;
            task->id=data[TASK_JSON_FIELD_ID];
            task->name=data[TASK_JSON_FIELD_NAME];
            task->type=data[TASK_JSON_FIELD_TYPE];
            task->repeatType=data[TASK_JSON_FIELD_REPEAT_TYPE];
            for(auto day:data[TASK_JSON_FIELD_DAY]){
                task->days.push_back(day);
            }
            string& startTime=data[TASK_JSON_FIELD_TIME].get_ref<string&>();
            if(strptime(startTime.c_str(),TASK_JSON_TIME_FORMAT,&task->startTime)!=NULL){
                task->expire=data[TASK_JSON_FIELD_EXPIRE];
                task->param=data[TASK_JSON_FIELD_PARAM].dump();
                task->notice=data[TASK_JSON_FIELD_NOTICE];
                task->active=data[TASK_JSON_FIELD_ACTIVE];
                task->schedued=false;
            }else{
                task=nullptr;
            }
        }catch(...){
            PLOG_WARN(TIMER_TASK_TAG,"invalid timer task data[%s]\n",path.c_str());
            task=nullptr;
        }
        return task;
    }
    int TimerTaskScheduler::saveTask(TimerTaskPtr task)
    {
        nlohmann::json data;
        try{
            data[TASK_JSON_FIELD_ID]=task->id;
            data[TASK_JSON_FIELD_NAME]=task->name;
            data[TASK_JSON_FIELD_TYPE]=task->type;
            data[TASK_JSON_FIELD_REPEAT_TYPE]=task->repeatType;
            data[TASK_JSON_FIELD_DAY]=nlohmann::json::array();
            auto &days=data[TASK_JSON_FIELD_DAY];
            for(auto &day:task->days){
                days.push_back(day);
            }
            char buf[64];
            strftime(buf,sizeof(buf),TASK_JSON_TIME_FORMAT,&task->startTime);
            data[TASK_JSON_FIELD_TIME]=buf;
            data[TASK_JSON_FIELD_EXPIRE]=task->expire;
            data[TASK_JSON_FIELD_PARAM]=nlohmann::json::parse(task->param);
            data[TASK_JSON_FIELD_NOTICE]=task->notice;
            data[TASK_JSON_FIELD_ACTIVE]=task->active;
        }catch(...){
            PLOG_WARN(TIMER_TASK_TAG,"invalid task param[%s]\n",task->param.c_str());
            return -1;
        }
        string path=mRootPath+to_string(task->id);
        std::ofstream out(path);
        out.width(1);//pretty print
        out<<data<<std::flush;
        return out.good()?0:-1;
    }
     int TimerTaskScheduler::genID()
     {
         int id=0;
gen:      
        id=(rand()%TASK_MAX_ID)+1;
        for(auto& task:mTask){
            if(id==task->id){
                goto gen;
            }
        }
         return id;
     }
     int TimerTaskScheduler::getDaySec(struct tm *t)
     {
         return t->tm_hour*3600+t->tm_min*60+t->tm_sec;
     }
     bool TimerTaskScheduler::findDay(TimerTaskPtr& task,int day)
     {
         return std::find(task->days.begin(),task->days.end(),day)!=task->days.end();
     }
     bool TimerTaskScheduler::sameDay(struct tm *l,struct tm* r)
     {
        PLOG_INFO(TIMER_TASK_TAG,"task %d %d %d %d %d %d", 
             l->tm_year,r->tm_year, l->tm_mon,r->tm_mon,l->tm_mday,r->tm_mday);
         return (l->tm_year==r->tm_year && l->tm_mon==r->tm_mon && l->tm_mday==r->tm_mday);
     }
}