/*
 * This file is part of the zlog Library.
 *
 * Copyright (C) 2011 by Hardy Simpson <HardySimpson1984@gmail.com>
 *
 * Licensed under the LGPL v2.1, see the file COPYING in base directory.
 */

#ifndef __zlog_h
#define __zlog_h

#ifdef __cplusplus
extern "C" {
#endif

#include <stdarg.h> /* for va_list */
#include <stdio.h> /* for size_t */

#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <pthread.h>
#include <time.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <fcntl.h>
#include <stdint.h>

#include <libgen.h>

#include <sys/select.h>    
#include <sys/inotify.h>   

# if defined __GNUC__
#   define ZLOG_CHECK_PRINTF(m,n) __attribute__((format(printf,m,n)))
# else 
#   define ZLOG_CHECK_PRINTF(m,n)
# endif

typedef struct zlog_category_s zlog_category_t;

int zlog_init(const char *config);
int zlog_reload(const char *config);
void zlog_fini(void);

void zlog_profile(void);

zlog_category_t *zlog_get_category(const char *cname);
int zlog_level_enabled(zlog_category_t *category, const int level);

int zlog_put_mdc(const char *key, const char *value);
char *zlog_get_mdc(const char *key);
void zlog_remove_mdc(const char *key);
void zlog_clean_mdc(void);

int zlog_level_switch(zlog_category_t * category, int level);
int zlog_level_enabled(zlog_category_t * category, int level);

void zlog(zlog_category_t * category,
	const char *file, size_t filelen,
	const char *func, size_t funclen,
	long line, int level,
	const char *format, ...) ZLOG_CHECK_PRINTF(8,9);
void vzlog(zlog_category_t * category,
	const char *file, size_t filelen,
	const char *func, size_t funclen,
	long line, int level,
	const char *format, va_list args);
void hzlog(zlog_category_t * category,
	const char *file, size_t filelen,
	const char *func, size_t funclen,
	long line, int level,
	const void *buf, size_t buflen);

int dzlog_init(const char *confpath, const char *cname);
int dzlog_set_category(const char *cname);

void dzlog(const char *file, size_t filelen,
	const char *func, size_t funclen,
	long line, int level,
	const char *format, ...) ZLOG_CHECK_PRINTF(7,8);
void vdzlog(const char *file, size_t filelen,
	const char *func, size_t funclen,
	long line, int level,
	const char *format, va_list args);
void hdzlog(const char *file, size_t filelen,
	const char *func, size_t funclen,
	long line, int level,
	const void *buf, size_t buflen);

typedef struct zlog_msg_s {
	char *buf;
	size_t len;
	char *path;
} zlog_msg_t;

typedef int (*zlog_record_fn)(zlog_msg_t *msg);
int zlog_set_record(const char *rname, zlog_record_fn record);

const char *zlog_version(void);

/******* useful macros, can be redefined at user's h file **********/

typedef enum {
	ZLOG_LEVEL_DEBUG = 20,
	ZLOG_LEVEL_INFO = 40,
	ZLOG_LEVEL_NOTICE = 60,
	ZLOG_LEVEL_WARN = 80,
	ZLOG_LEVEL_ERROR = 100,
	ZLOG_LEVEL_FATAL = 120
} zlog_level; 

#define ESC_START       "\033["
#define ESC_END         "	\033[0m"
#define COLOR_FATAL     "31;40;5m "
#define COLOR_ALERT     "31;40;1m "
#define COLOR_CRIT      "31;40;1m "
#define COLOR_ERROR     "35;40;1m "
#define COLOR_WARN      "33;40;1m "
#define COLOR_NOTICE    "34;40;1m "
#define COLOR_INFO      "32;40;1m "
#define COLOR_DEBUG     "36;40;1m "
#define COLOR_TRACE     "37;40;1m "


#if !defined(__STDC_VERSION__) || __STDC_VERSION__ < 199901L
# if defined __GNUC__ && __GNUC__ >= 2
#  define __func__ __FUNCTION__
# else
#  define __func__ "<unknown>"
# endif
#endif

#if defined __STDC_VERSION__ && __STDC_VERSION__ >= 199901L
/* zlog macros */
#define zlog_fatal(cat, ...) \
	zlog(cat, __FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_FATAL, __VA_ARGS__)
#define zlog_error(cat, ...) \
	zlog(cat, __FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_ERROR, __VA_ARGS__)
#define zlog_warn(cat, ...) \
	zlog(cat, __FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_WARN, __VA_ARGS__)
#define zlog_notice(cat, ...) \
	zlog(cat, __FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_NOTICE, __VA_ARGS__)
#define zlog_info(cat, ...) \
	zlog(cat, __FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_INFO, __VA_ARGS__)
#define zlog_debug(cat, ...) \
	zlog(cat, __FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_DEBUG, __VA_ARGS__)
/* dzlog macros */
#define dzlog_fatal(...) \
	dzlog(__FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_FATAL, __VA_ARGS__)
#define dzlog_error(...) \
	dzlog(__FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_ERROR, __VA_ARGS__)
#define dzlog_warn(...) \
	dzlog(__FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_WARN, __VA_ARGS__)
#define dzlog_notice(...) \
	dzlog(__FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_NOTICE, __VA_ARGS__)
#define dzlog_info(...) \
	dzlog(__FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_INFO, __VA_ARGS__)
#define dzlog_debug(...) \
	dzlog(__FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_DEBUG, __VA_ARGS__)
#elif defined __GNUC__


/* zlog macros */
#define zlog_fatal(cat, format, args...) \
	zlog(cat, __FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_FATAL, format, ##args)
#define zlog_error(cat, format, args...) \
	zlog(cat, __FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_ERROR, format, ##args)
#define zlog_warn(cat, format, args...) \
	zlog(cat, __FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_WARN, format, ##args)
#define zlog_notice(cat, format, args...) \
	zlog(cat, __FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_NOTICE, format, ##args)
#define zlog_info(cat, format, args...) \
	zlog(cat, __FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_INFO, format, ##args)
#define zlog_debug(cat, format, args...) \
	zlog(cat, __FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_DEBUG, format, ##args)
/* dzlog macros */
#define dzlog_fatal(format, args...) \
	dzlog(__FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_FATAL, ESC_START COLOR_FATAL format ESC_END, ##args)
#define dzlog_error(format, args...) \
	dzlog(__FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_ERROR, ESC_START COLOR_ERROR format ESC_END, ##args)
#define dzlog_warn(format, args...) \
	dzlog(__FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_WARN, ESC_START COLOR_WARN format ESC_END, ##args)
#define dzlog_notice(format, args...) \
	dzlog(__FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_NOTICE, ESC_START COLOR_NOTICE format ESC_END, ##args)
#define dzlog_info(format, args...) \
	dzlog(__FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_INFO, ESC_START COLOR_INFO format ESC_END, ##args)
#define dzlog_debug(format, args...) \
	dzlog(__FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_DEBUG, ESC_START COLOR_DEBUG format ESC_END, ##args)
#endif

/* vzlog macros */
#define vzlog_fatal(cat, format, args) \
	vzlog(cat, __FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_FATAL, format, args)
#define vzlog_error(cat, format, args) \
	vzlog(cat, __FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_ERROR, format, args)
#define vzlog_warn(cat, format, args) \
	vzlog(cat, __FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_WARN, format, args)
#define vzlog_notice(cat, format, args) \
	vzlog(cat, __FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_NOTICE, format, args)
#define vzlog_info(cat, format, args) \
	vzlog(cat, __FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_INFO, format, args)
#define vzlog_debug(cat, format, args) \
	vzlog(cat, __FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_DEBUG, format, args)

/* hzlog macros */
#define hzlog_fatal(cat, buf, buf_len) \
	hzlog(cat, __FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_FATAL, buf, buf_len)
#define hzlog_error(cat, buf, buf_len) \
	hzlog(cat, __FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_ERROR, buf, buf_len)
#define hzlog_warn(cat, buf, buf_len) \
	hzlog(cat, __FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_WARN, buf, buf_len)
#define hzlog_notice(cat, buf, buf_len) \
	hzlog(cat, __FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_NOTICE, buf, buf_len)
#define hzlog_info(cat, buf, buf_len) \
	hzlog(cat, __FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_INFO, buf, buf_len)
#define hzlog_debug(cat, buf, buf_len) \
	hzlog(cat, __FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_DEBUG, buf, buf_len)


/* vdzlog macros */
#define vdzlog_fatal(format, args) \
	vdzlog(__FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_FATAL, format, args)
#define vdzlog_error(format, args) \
	vdzlog(__FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_ERROR, format, args)
#define vdzlog_warn(format, args) \
	vdzlog(__FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_WARN, format, args)
#define vdzlog_notice(format, args) \
	vdzlog(__FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_NOTICE, format, args)
#define vdzlog_info(format, args) \
	vdzlog(__FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_INFO, format, args)
#define vdzlog_debug(format, args) \
	vdzlog(__FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_DEBUG, format, args)

/* hdzlog macros */
#define hdzlog_fatal(buf, buf_len) \
	hdzlog(__FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_FATAL, buf, buf_len)
#define hdzlog_error(buf, buf_len) \
	hdzlog(__FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_ERROR, buf, buf_len)
#define hdzlog_warn(buf, buf_len) \
	hdzlog(__FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_WARN, buf, buf_len)
#define hdzlog_notice(buf, buf_len) \
	hdzlog(__FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_NOTICE, buf, buf_len)
#define hdzlog_info(buf, buf_len) \
	hdzlog(__FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_INFO, buf, buf_len)
#define hdzlog_debug(buf, buf_len) \
	hdzlog(__FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_DEBUG, buf, buf_len)

/* enabled macros */
#define zlog_fatal_enabled(zc) zlog_level_enabled(zc, ZLOG_LEVEL_FATAL)
#define zlog_error_enabled(zc) zlog_level_enabled(zc, ZLOG_LEVEL_ERROR)
#define zlog_warn_enabled(zc) zlog_level_enabled(zc, ZLOG_LEVEL_WARN)
#define zlog_notice_enabled(zc) zlog_level_enabled(zc, ZLOG_LEVEL_NOTICE)
#define zlog_info_enabled(zc) zlog_level_enabled(zc, ZLOG_LEVEL_INFO)
#define zlog_debug_enabled(zc) zlog_level_enabled(zc, ZLOG_LEVEL_DEBUG)
/*rmwei*/
typedef struct {
	char *confpath;
	char *levelpath;
	char *logpath;
	char *cname;
}log_t;

int dzlogInit(log_t *arg, int size);
int dzlogfInit();
extern log_t *log_handle;

#ifndef PLOG_FATAL
#define PLOG_FATAL(tag,format,args...) \
	dzlog(__FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_FATAL, ESC_START COLOR_FATAL format ESC_END, ##args)
#else
#undef PLOG_FATAL
#define PLOG_FATAL(tag,format,args...) \
	dzlog(__FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_FATAL, ESC_START COLOR_FATAL format ESC_END, ##args)
				
#endif
#ifndef PLOG_ERROR
#define PLOG_ERROR(tag,format,args...) \
	dzlog(__FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_ERROR, ESC_START COLOR_ERROR format ESC_END, ##args)
#else
#undef PLOG_ERROR
#define PLOG_ERROR(tag,format,args...) \
	dzlog(__FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_ERROR, ESC_START COLOR_ERROR format ESC_END, ##args)
				
#endif
#ifndef PLOG_WARN
#define PLOG_WARN(tag,format, args...) \
	dzlog(__FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_WARN, ESC_START COLOR_WARN format ESC_END, ##args)
#else
#undef PLOG_WARN
#define PLOG_WARN(tag,format, args...) \
	dzlog(__FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_WARN, ESC_START COLOR_WARN format ESC_END, ##args)
				
#endif
#ifndef PLOG_NOTICE
#define PLOG_NOTICE(tag,format, args...) \
	dzlog(__FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_NOTICE, ESC_START COLOR_NOTICE format ESC_END, ##args)
#else
#undef PLOG_NOTICE
#define PLOG_NOTICE(tag,format, args...) \
	dzlog(__FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_NOTICE, ESC_START COLOR_NOTICE format ESC_END, ##args)
#endif
#ifndef PLOG_INFO
#define PLOG_INFO(tag,format, args...) \
	dzlog(__FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_INFO, ESC_START COLOR_INFO format ESC_END, ##args)
#else
#undef PLOG_INFO
#define PLOG_INFO(tag,format, args...) \
	dzlog(__FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_INFO, ESC_START COLOR_INFO format ESC_END, ##args)
				
#endif
#ifndef PLOG_DEBUG
#define PLOG_DEBUG(tag,format, args...) \
	dzlog(__FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_DEBUG, ESC_START COLOR_DEBUG format ESC_END, ##args)
#else
#undef PLOG_DEBUG
#define PLOG_DEBUG(tag,format, args...) \
	dzlog(__FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_DEBUG, ESC_START COLOR_DEBUG format ESC_END, ##args)
				
#endif


#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
#include<math.h>
#include<thread>
#include<mutex>
#include<unistd.h>
#include<fstream>
#include<iostream>
using namespace std;

class StdZlog{
public:
	StdZlog()
	{
		Cname = ReadLink();
		ConfPath = "/var/roller_eye/config/log/" + Cname + ".cfg";
		LevelPath = "/var/roller_eye/config/log/log.level";
		LogPath = "/var/log/node/" + Cname + ".log";
		
		log_t arg = {
			confpath:	(char *)ConfPath.c_str(),
			levelpath:	(char *)LevelPath.c_str(),
			logpath:	(char *)LogPath.c_str(),
			cname:		(char *)Cname.c_str()
		};
		if(0 != dzlogInit(&arg,2)){
			printf("%s log int error.\r\n",(char *)Cname.c_str());
		}

	}
	~StdZlog()
	{
		dzlogfInit();
	}
	std::string ReadLink(){
		char name[100];
		int rval = readlink("/proc/self/exe",name,sizeof(name)-1);
		if(rval == -1){
			cout << "readlink error" << endl;
		}
		name[rval] = '\0';
		return string(strrchr(name, '/') + 1);
	}	
private:
	std::string ConfPath;
	std::string LevelPath;
	std::string LogPath;
	std::string Cname;
} ;
	
//static shared_ptr<StdZlog> pStdZlog = NULL;	
#ifndef PLOG_FATAL
#define PLOG_FATAL(tag,format,args...) \
	{if(NULL == log_handle){StdZlog *log =new StdZlog();} \
	dzlog(__FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_FATAL, ESC_START COLOR_FATAL format ESC_END, ##args);}
#else
#undef PLOG_FATAL
#define PLOG_FATAL(tag,format,args...) \
	{if(NULL == log_handle){StdZlog *log =new StdZlog();} \
	dzlog(__FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_FATAL, ESC_START COLOR_FATAL format ESC_END, ##args);}
		
#endif
#ifndef PLOG_ERROR
#define PLOG_ERROR(tag,format,args...) \
	{if(NULL == log_handle){StdZlog *log =new StdZlog();} \
	dzlog(__FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_ERROR, ESC_START COLOR_ERROR format ESC_END, ##args);}
#else
#undef PLOG_ERROR
#define PLOG_ERROR(tag,format,args...) \
	{if(NULL == log_handle){StdZlog *log =new StdZlog();} \
	dzlog(__FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_ERROR, ESC_START COLOR_ERROR format ESC_END, ##args);}
		
#endif
#ifndef PLOG_WARN
#define PLOG_WARN(tag,format, args...) \
	{if(NULL == log_handle){StdZlog *log =new StdZlog();} \
	dzlog(__FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_WARN, ESC_START COLOR_WARN format ESC_END, ##args);}
#else
#undef PLOG_WARN
#define PLOG_WARN(tag,format, args...) \
	{if(NULL == log_handle){StdZlog *log =new StdZlog();} \
	dzlog(__FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_WARN, ESC_START COLOR_WARN format ESC_END, ##args);}
		
#endif
#ifndef PLOG_NOTICE
#define PLOG_NOTICE(tag,format, args...) \
	{if(NULL == log_handle){StdZlog *log =new StdZlog();} \
	dzlog(__FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_NOTICE, ESC_START COLOR_NOTICE format ESC_END, ##args);}
#else
#undef PLOG_NOTICE
#define PLOG_NOTICE(tag,format, args...) \
	{if(NULL == log_handle){StdZlog *log =new StdZlog();} \
	dzlog(__FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_NOTICE, ESC_START COLOR_NOTICE format ESC_END, ##args);}
#endif
#ifndef PLOG_INFO
#define PLOG_INFO(tag,format, args...) \
	{if(NULL == log_handle){StdZlog *log =new StdZlog();} \
	dzlog(__FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_INFO, ESC_START COLOR_INFO format ESC_END, ##args);}
#else
#undef PLOG_INFO
#define PLOG_INFO(tag,format, args...) \
	{if(NULL == log_handle){StdZlog *log =new StdZlog();} \
	dzlog(__FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_INFO, ESC_START COLOR_INFO format ESC_END, ##args);}
		
#endif
#ifndef PLOG_DEBUG
#define PLOG_DEBUG(tag,format, args...) \
	{if(NULL == log_handle){StdZlog *log =new StdZlog();} \
	dzlog(__FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_DEBUG, ESC_START COLOR_DEBUG format ESC_END, ##args);}
#else
#undef PLOG_DEBUG
#define PLOG_DEBUG(tag,format, args...) \
	{if(NULL == log_handle){StdZlog *log =new StdZlog();} \
	dzlog(__FILE__, sizeof(__FILE__)-1, __func__, sizeof(__func__)-1, __LINE__, \
	ZLOG_LEVEL_DEBUG, ESC_START COLOR_DEBUG format ESC_END, ##args);}
		
#endif

#endif

#endif
