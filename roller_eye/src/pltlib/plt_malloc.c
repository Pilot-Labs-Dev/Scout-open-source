#include"plt_malloc.h"
#include"plog.h"
#include"plterrno.h"
#include<stdlib.h>

static const char* PMALLOC_TAG="ptl_malloc";

#define PROC_ERROR(ptr)\
if(ptr==NULL){\
        __proc_error(func,line,flag);\
}

static void __proc_error(const char* func,int line,int flag)
{
    pset_errno(PEMEM);
    
    if(flag&PMALLOC_FLAG_FAIL_LOG){
        __plog_write(PMALLOC_TAG,PLOG_LEVEL_FATAL,PLOG_FLAG_ERRNO|PLOG_FLAG_POS,func,line,"malloc fail\n");
    }
    if(flag&PMALLOC_FLAG_FAIL_EXIT){
        __plog_write(PMALLOC_TAG,PLOG_LEVEL_FATAL,PLOG_FLAG_ERRNO|PLOG_FLAG_POS|PLOG_FLAG_SYSLOG|PLOG_FLAG_TIME,func,line,"malloc fail\n");
        exit(1);
    }
}
void* __plt_malloc(size_t size,const char *func,int line,int flag)
{
    void *ptr=malloc(size);
    PROC_ERROR(ptr);
    return ptr;
}
void* __plt_calloc(size_t n,size_t size,const char *func,int line,int flag)
{
    void *ptr=calloc(n,size);
    PROC_ERROR(ptr);
    return ptr;
}
void *__plt_realloc(void *ptr, size_t size,const char *func,int line,int flag)
{
    void *ptr2=realloc(ptr,size);
    PROC_ERROR(ptr2);
    return ptr2;
}
void *__plt_reallocarray(void *ptr, size_t n, size_t size,const char *func,int line,int flag)
{
#ifdef _GNU_SOURCE
    void *ptr2=reallocarray(ptr,n,size);
#else
    void *ptr2=realloc(ptr,n*size);
#endif
    PROC_ERROR(ptr2);
    return ptr2;
}

void plt_free(void *ptr){
    free(ptr);
}

