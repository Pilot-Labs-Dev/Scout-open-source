#ifndef __PLOG_PLUS_H__
#define __PLOG_PLUS_H__

#include"platfarm_define.h"

#define PLOG_FLAG_POS                0x00000001    //print function name and line
#define PLOG_FLAG_TIME               0x00000002     //print system time
#define PLOG_FLAG_SYSLOG         0x00000004     //log to system log
#define PLOG_FLAG_ERRNO          0x00000008     //print errno 

enum{
            PLOG_LEVEL_DEBUG=1,             
            PLOG_LEVEL_INFO,                  
            PLOG_LEVEL_WARNING,      
            PLOG_LEVEL_ERROR,              
            PLOG_LEVEL_FATAL,
            PLOG_LEVEL_MAX
};                

#ifdef __cplusplus
extern "C"{
#endif
void plog_init(int level);
void plog_setlevel(int level);
int __plog_write(const char* tag,int level,int flags,const char* fun,int line,const char* fmt,...);
#ifdef __cplusplus
}
#endif
#define plog_write(tag,level,flags,fmt,...)  __plog_write(tag,level,flags,__FUNCTION__,__LINE__,fmt,##__VA_ARGS__)

#ifdef USE_ROS //use ros log system
#include<ros/ros.h>
#include<string>
#include"plterrno.h"
#endif
#include "zlog.h"
#if 0
#ifdef ROLLER_DEBUG
#ifdef USE_ROS
//same as printf
#define PLOG_PRINTF(fmt,...)            ROS_DEBUG(fmt,##__VA_ARGS__)
//out put to console
#define PLOG_DEBUG(tag,fmt,...)     ROS_DEBUG(fmt,##__VA_ARGS__)
#define PLOG_INFO(tag,fmt,...)          ROS_INFO(fmt,##__VA_ARGS__)
#define PLOG_WARN(tag,fmt,...)        ROS_WARN(fmt,##__VA_ARGS__)
#define PLOG_ERROR(tag,fmt,...)       do{std::string format="reason[%s] ";format+=fmt;ROS_ERROR(format.c_str(),pstr_lasterr(),##__VA_ARGS__);}while(0)
#define PLOG_FATAL(tag,fmt,...)          do{std::string format="reason[%s] ";format+=fmt;ROS_FATAL(format.c_str(),pstr_lasterr(),##__VA_ARGS__);}while(0)

#define PLOG_DEBUG_T(tag,fmt,...)     ROS_DEBUG(fmt,##__VA_ARGS__)
#define PLOG_INFO_T(tag,fmt,...)          ROS_INFO(fmt,##__VA_ARGS__)
#define PLOG_WARN_T(tag,fmt,...)        ROS_WARN(fmt,##__VA_ARGS__)
#define PLOG_ERROR_T(tag,fmt,...)      do{std::string format="reason[%s] ";format+=fmt;ROS_ERROR(format.c_str(),pstr_lasterr(),##__VA_ARGS__);}while(0)
#define PLOG_FATAL_T(tag,fmt,...)         do{std::string format="reason[%s] ";format+=fmt;ROS_FATAL(format.c_str(),pstr_lasterr(),##__VA_ARGS__);}while(0)

//Write info/system error to system log
#define PLOG_LOG_INFO(tag,fmt,...)   ROS_INFO(fmt,##__VA_ARGS__)
#define PLOG_LOG_ERROR(tag,fmt,...)  do{std::string format="reason[%s] ";format+=fmt;ROS_FATAL(format.c_str(),pstr_lasterr(),##__VA_ARGS__);}while(0)
#define ROS_ERROR2(fmt,...) PLOG_ERROR("",fmt,##__VA_ARGS__)  
#else
//same as printf
#define PLOG_PRINTF(fmt,...)            plog_write((char*)0,PLOG_LEVEL_INFO,0,fmt,##__VA_ARGS__)
//out put to console
#define PLOG_DEBUG(tag,fmt,...)     plog_write(tag,PLOG_LEVEL_DEBUG,PLOG_FLAG_POS,fmt,##__VA_ARGS__)
#define PLOG_INFO(tag,fmt,...)          plog_write(tag,PLOG_LEVEL_INFO,PLOG_FLAG_POS,fmt,##__VA_ARGS__)
#define PLOG_WARN(tag,fmt,...)        plog_write(tag,PLOG_LEVEL_WARNING,PLOG_FLAG_POS,fmt,##__VA_ARGS__)
#define PLOG_ERROR(tag,fmt,...)     plog_write(tag,PLOG_LEVEL_ERROR,PLOG_FLAG_POS|PLOG_FLAG_ERRNO,fmt,##__VA_ARGS__)
#define PLOG_FATAL(tag,fmt,...)        plog_write(tag,PLOG_LEVEL_FATAL,PLOG_FLAG_POS|PLOG_FLAG_ERRNO,fmt,##__VA_ARGS__)

#define PLOG_DEBUG_T(tag,fmt,...)     plog_write(tag,PLOG_LEVEL_DEBUG,PLOG_FLAG_POS|PLOG_FLAG_TIME,fmt,##__VA_ARGS__)
#define PLOG_INFO_T(tag,fmt,...)          plog_write(tag,PLOG_LEVEL_INFO,PLOG_FLAG_POS|PLOG_FLAG_TIME,fmt,##__VA_ARGS__)
#define PLOG_WARN_T(tag,fmt,...)        plog_write(tag,PLOG_LEVEL_WARNING,PLOG_FLAG_POS|PLOG_FLAG_TIME,fmt,##__VA_ARGS__)
#define PLOG_ERROR_T(tag,fmt,...)     plog_write(tag,PLOG_LEVEL_ERROR,PLOG_FLAG_POS|PLOG_FLAG_ERRNO|PLOG_FLAG_TIME,fmt,##__VA_ARGS__)
#define PLOG_FATAL_T(tag,fmt,...)         plog_write(tag,PLOG_LEVEL_FATAL,PLOG_FLAG_POS|PLOG_FLAG_ERRNO|PLOG_FLAG_TIME,fmt,##__VA_ARGS__)

//Write info/system error to system log
#define PLOG_LOG_INFO(tag,fmt,...)   plog_write(tag,PLOG_LEVEL_MAX,PLOG_FLAG_POS|PLOG_FLAG_SYSLOG,fmt,##__VA_ARGS__)
#define PLOG_LOG_ERROR(tag,fmt,...)   plog_write(tag,PLOG_LEVEL_ERROR,PLOG_FLAG_POS|PLOG_FLAG_SYSLOG|PLOG_FLAG_ERRNO,fmt,##__VA_ARGS__)
#endif

#else
#define PLOG_PRINTF(fmt,...)          
#define PLOG_DEBUG(tag,fmt,...)     
#define PLOG_INFO(tag,fmt,...)       
#define PLOG_WARN(tag,fmt,...)      
#define PLOG_ERROR(tag,fmt,...)      
#define PLOG_FATAL(tag,fmt,...)          

#define PLOG_DEBUG_T(tag,fmt,...)     
#define PLOG_INFO_T(tag,fmt,...)          
#define PLOG_WARN_T(tag,fmt,...)        
#define PLOG_ERROR_T(tag,fmt,...)      
#define PLOG_FATAL_T(tag,fmt,...)        

#define PLOG_LOG_INFO(tag,fmt,...)  
#define PLOG_LOG_ERROR(tag,fmt,...)  
#define ROS_DEBUG(fmt,...)       
#define ROS_INFO(fmt,...)   
#define ROS_WARN(fmt,...)       
#define ROS_ERROR(fmt,...)       
#define ROS_ERROR2(fmt,...)       

#endif
#endif
#endif