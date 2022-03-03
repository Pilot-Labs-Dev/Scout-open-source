#ifndef __PLOG__H__
#define __PLOG__H__
#ifdef __cplusplus
extern "C"{
#endif

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


void plog_init(int level);
void plog_setlevel(int level);
int __plog_write(const char* tag,int level,int flags,const char* fun,int line,const char* fmt,...);

#define plog_write(tag,level,flags,fmt,...)  __plog_write(tag,level,flags,__FUNCTION__,__LINE__,fmt,##__VA_ARGS__)
#include "zlog.h"
#if 0
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
#ifdef __cplusplus
}
#endif
#endif