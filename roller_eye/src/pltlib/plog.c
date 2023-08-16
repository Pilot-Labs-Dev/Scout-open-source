#include<stdio.h>
#include<sys/time.h>
#include<string.h>
#include <syslog.h>
#include<stdarg.h>
#include "plog.h"
#include"plterrno.h"
#include "zlog.h"
#include "errno.h"
int  zlog_flag = 0;
log_t *log_handle = NULL;
pthread_mutex_t log_mutex = PTHREAD_MUTEX_INITIALIZER; 


static void   zlog_event_handler(struct inotify_event *event,log_t *logarg)
{	
	if(event->mask & IN_MODIFY) {
		//printf("IN_MODIFY.zlog_cname:%s\n",zlog_cname);
		zlog_category_t *zc = zlog_get_category(logarg->cname);
		if (zc) {
			FILE * fd;
			int level = 0;
			char line_buf[32] = {0};
			if ( NULL == (fd=fopen(logarg->levelpath,"r")) ){
				printf ("open %s Error",logarg->levelpath);
				return ;
			}
			while(NULL != fgets(line_buf,32,fd)){
				level = atoi(line_buf);
				
				printf("log level:%d\n",level);
				zlog_level_switch(zc,level);
			}
		}else{
			printf("zlog_get_category is NULL.\r\n");
		}
	}
}

static void  zlog_set_notify(log_t *logarg/*char *config,char * cname*/)   {
	unsigned char buf[1024] = {0};   
	struct inotify_event *event = NULL;	
	
	int fd = inotify_init();
	int wd = inotify_add_watch(fd, logarg->levelpath, IN_MODIFY/*IN_ALL_EVENTS*/);	
	zlog_flag = 1;
	printf("zlog_set_notify, %s,%d\r\n",logarg->levelpath,zlog_flag);
	while (zlog_flag) { 
		fd_set fds;	 
		FD_ZERO(&fds);				 
		FD_SET(fd, &fds);   
		if (select(fd + 1, &fds, NULL, NULL, NULL) > 0){
			int len, index = 0;	 
			while (((len = read(fd, &buf, sizeof(buf))) < 0) && (errno == EINTR));
			while (index < len) {   
				event = (struct inotify_event *)(buf + index);
				zlog_event_handler(event,logarg);
				index += sizeof(struct inotify_event) + event->len;
			}   
		}   
	}
	inotify_rm_watch(fd, wd);
}  

static void zlog_del_notify(){
	zlog_flag = 0;
	printf("zlog_del_notify ok.\r\n");
}

static int WriteCfg(char *confpath, char *logpath,char *cname, int size)
{
	 if(access(confpath, F_OK) != 0){
		int eth_fd;
		char buf[256] = {0};

		char *dirc = strdup(confpath);
		char *path_name = dirname(dirc);

		if(access(path_name, F_OK) != 0){
			char cmd[128] = {0};
			sprintf(cmd,"mkdir -p %s",path_name);
			system(cmd);
		}
		
		if(NULL != dirc){
			free(dirc);
		}
		if((eth_fd = open(confpath,O_CREAT|O_RDWR|O_TRUNC,S_IRWXU)) < 0){
			printf("open %s Error",confpath);
			return -1;
		}
		//printf("buf0:%s\r\n",buf);
		char *strbuf = buf;
		strbuf+=sprintf(strbuf,"%s","[formats]\n");
		strbuf+=sprintf(strbuf,"%s","default = \"\%d(\%m-\%d \%T) \%-5V [\%p:\%f:\%L:\%U] \%m\%n\"\n");
		strbuf+=sprintf(strbuf,"%s","[rules]\n");
		strbuf+=sprintf(strbuf,"%s.* >stderr;default\n",cname);
		strbuf+=sprintf(strbuf,"%s.DEBUG \"%s\", %dM*1;default\n",cname,logpath,size);
		//strbuf+=sprintf(strbuf,"%s.DEBUG \"%s\", 2M*1;default\n",cname,logpath);

		lseek(eth_fd,0,SEEK_SET);
		//printf("buf:%s\r\n",buf);
		write(eth_fd,buf,strlen(buf));
		close(eth_fd);
	 }

	return 0;
}

static void* zlog_thread_loop(void* arg){
	log_t *logarg = (log_t *)arg;
	if(NULL == arg){
		printf("zlog_thread_loop arg is NULL!\r\n");
	}
			
	pthread_detach(pthread_self());
	if(access(logarg->levelpath, F_OK) != 0){
		int eth_fd;
		char buf[128] = {0};
		char *dirc = strdup(logarg->levelpath);
		char *path_name = dirname(dirc);
		if(access(path_name, F_OK) != 0){
			char cmd[128] = {0};
			sprintf(cmd,"mkdir -p %s",path_name);
			system(cmd);
		}
		if(NULL != dirc){
			free(dirc);
		}
		if((eth_fd = open(logarg->levelpath,O_CREAT|O_RDWR|O_TRUNC,S_IRWXU)) < 0){
			printf("open %s Error",logarg->levelpath);
		}
		sprintf(buf,"100\n");
		lseek(eth_fd,0,SEEK_SET);
		write(eth_fd,buf,strlen(buf));
		close(eth_fd);
		zlog_category_t *zc = zlog_get_category(logarg->cname);
		if (zc) {
			zlog_level_switch(zc,100);
		}
	}else{
		zlog_category_t *zc = zlog_get_category(logarg->cname);
		if (zc) {
			FILE * fd;
			int level = 0;
			char line_buf[32] = {0};
			if ( NULL == (fd=fopen(logarg->levelpath,"r")) ){
				printf ("open %s Error",logarg->levelpath);
			}else{
				while(NULL != fgets(line_buf,32,fd)){
					level = atoi(line_buf);
					
					printf("level:%d\r\n",level);
					zlog_level_switch(zc,level);
				}
			}
		}
	}

	zlog_set_notify(logarg);
}

void dzlog_mkdir(log_t *arg){
	char *confpath_dirc = strdup(arg->confpath);
	char *levelpath_dirc = strdup(arg->levelpath);
	char *logpath_dirc = strdup(arg->logpath);
	
	char *confpath = dirname(confpath_dirc);
	char *logpath = dirname(logpath_dirc);
	char *levelpath = dirname(levelpath_dirc);
	
	if(access(confpath, F_OK) != 0){
		char cmd[128] = {0};
		sprintf(cmd,"mkdir -p %s",confpath);
		system(cmd);
	}

	if(access(logpath, F_OK) != 0){
		char cmd[128] = {0};
		sprintf(cmd,"mkdir -p %s",logpath);
		system(cmd);
	}

	if(access(levelpath, F_OK) != 0){
		char cmd[128] = {0};
		sprintf(cmd,"mkdir -p %s",levelpath);
		system(cmd);
	}
	if(NULL  != confpath_dirc){
		free(confpath_dirc);
		confpath_dirc =NULL;
	}
	if(NULL  != levelpath_dirc){
		free(levelpath_dirc);
		levelpath_dirc =NULL;
	}
	if(NULL  != logpath_dirc){
		free(logpath_dirc);
		logpath_dirc =NULL;
	}
}

int dzlogInit(log_t *arg, int size){
	int ret = 0;
	pthread_mutex_lock(&log_mutex);
	if(log_handle){
		//free(log_handle);
		dzlogfInit();
	}
	log_handle = (log_t *)malloc(sizeof(log_t));
	
	if(log_handle){
		dzlog_mkdir(arg);
		memcpy(log_handle,arg,sizeof(log_t));

		printf("dzlogInit %s %s %s!\r\n",log_handle->confpath, log_handle->logpath, log_handle->cname );
		static pthread_t zlog_thread;
		zlog_flag = 0;
		ret+= WriteCfg(log_handle->confpath,log_handle->logpath,log_handle->cname, size);
		ret+= dzlog_init(log_handle->confpath, log_handle->cname);
		ret+= pthread_create(&zlog_thread,NULL,zlog_thread_loop,(void*)log_handle);
	}else{
		printf("malloc size:%d error!\r\n",sizeof(log_t));
		ret =-1;
	}
	pthread_mutex_unlock(&log_mutex);
	return ret;
}

int dzlogfInit(){
	zlog_del_notify();
	zlog_fini();
	if(log_handle){
		free(log_handle);
		log_handle = NULL;
	}
	return 0;
}



#if 1
static int g_level=PLOG_LEVEL_INFO;
static int g_inited=0;

void plog_init(int level)
{
    if(!g_inited){
        g_inited=1;
       plog_setlevel(level);
        openlog(NULL,LOG_PID|LOG_CONS,LOG_USER);
    }else{
        //fprintf(stderr,"log module has been inited\n");
    }
    
}
void plog_setlevel(int level)
{
    if(level>=PLOG_LEVEL_DEBUG&&level<PLOG_LEVEL_MAX){
        g_level=level;
    }else{
        fprintf(stderr,"LOG Level is out of range\n");
    }
}
static int mapLevel(int level)
{
        int syslevel;
        switch (level)
        {
        case PLOG_LEVEL_DEBUG:
            syslevel=LOG_DEBUG;
            break;
        case PLOG_LEVEL_INFO:
            syslevel=LOG_INFO;
            break;
        case PLOG_LEVEL_WARNING:
            syslevel=LOG_WARNING;
            break;
        case PLOG_LEVEL_ERROR:
            syslevel=LOG_ERR;
            break;
        case PLOG_LEVEL_FATAL:
            syslevel=LOG_CRIT;
            break;
        default:
            syslevel=LOG_NOTICE;
            break;
        }
        return syslevel;
}
int __plog_write(const char* tag,int level,int flags,const char* fun,int line,const char* fmt,...)
{
    int ret,len;
    va_list varlist;

    plog_init(g_level);

    if(!g_inited){
        fprintf(stderr,"LOG module do not init,call \"plog_init\" first\n");
        return 0;
    }
    
    if(level<g_level){
        return 0;
    }

     if((len=strlen(fmt))>512){
         fprintf(stderr,"fmt is too long,may be memory out of range\n");
         return 0;
     }
        
    char prefix[256];
    int pos=0;

    prefix[0]=0;
    if(tag!=NULL){
         pos=snprintf(prefix,sizeof(prefix),"[%s] ",tag);
    }
    
    if(flags&PLOG_FLAG_TIME){
        struct timeval val;
        if(gettimeofday(&val,NULL)==0){
            pos+=snprintf(prefix+pos,sizeof(prefix)-pos,"[%d.%d]",(int)val.tv_sec,(int)val.tv_usec);
        }
    }
    if(flags&PLOG_FLAG_POS){
        pos+=snprintf(prefix+pos,sizeof(prefix)-pos,"(%s:%d) ",fun,line);
    }

    if(flags&PLOG_FLAG_ERRNO){
        pos+=snprintf(prefix+pos,sizeof(prefix)-pos,"(errno %d:%s)",pget_errno(),pstr_errno(pget_errno()));
    }
    
    char fmtnew[strlen(prefix)+strlen(fmt)+1];
    sprintf(fmtnew,"%s%s",prefix,fmt);

    va_start (varlist,fmt);
    ret=vprintf(fmtnew,varlist);
    va_end(varlist);

    if(flags&PLOG_FLAG_SYSLOG){
        va_start (varlist,fmt);
        vsyslog(mapLevel(level),fmtnew,varlist);
        va_end(varlist);
    }
    return ret;
}
#endif
