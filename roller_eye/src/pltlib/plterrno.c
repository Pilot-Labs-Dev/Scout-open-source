#include<pthread.h>
#include<errno.h>
#include<string.h>
#include"plterrno.h"
#ifdef LIB_UBUS
#include<libubus.h>
#endif
#ifdef LIB_UV
#include<uv.h>
#endif
#include"common_macros.h"

static const char* g_err_reason[]={
    "reason not set",
    "",//system error
    "memory not enough",
    "null ptr",
    "this is a bug",
    "resource full",
    "resource not ready",
    "resource busy",
    "resource invalid",
    "invalid param",
    "blob error",
    "not found",
    "unkown error",
    "bad param",
    PERRMSG_TEST
};
static pthread_key_t g_err_key;
static pthread_once_t  g_once=PTHREAD_ONCE_INIT;

void free_key(void* ptr){
    free(ptr);
}
static void init_errno_key(void){
    pthread_key_create(&g_err_key,free_key);
}
void pset_errno(int no)
{
    pthread_once(&g_once,init_errno_key);
    int *err=(int*)pthread_getspecific(g_err_key);
    if(err==NULL){
        err=(int*)malloc(sizeof(int));
        if(err!=NULL && pthread_setspecific(g_err_key,err)!=0){
            free(err);
            err=NULL;
        }
    }
    if(err!=NULL)
        *err=no;
}
#ifdef LIB_UBUS
void pset_ubusno(int no)
{
    pset_errno(no+PEUBUS_START);
}
#endif
#ifdef LIB_UV
void pset_uvno(int no)
{
    pset_errno(-1*no+PEUV_START);
}
#endif
void pset_pthreadno(int no)
{
    pset_errno(no+PETHD_START);
}

int pget_errno(void)
{
    pthread_once(&g_once,init_errno_key);
     int *err=(int*)pthread_getspecific(g_err_key);
     return err==NULL?0:*err;
}
const char* pstr_errno(int no)
{
        if(no==PESYS||no==0){
            return strerror(errno);
        }
#ifdef LIB_UBUS
        else if(no>=PEUBUS_START&& no< PEUBUS_END){
            return ubus_strerror(no-PEUBUS_START);
        }
#endif
#ifdef LIB_UV
        else if(no>=PEUV_START&& no< PEUV_END){
            return uv_strerror(-1*(no-PEUV_START));
        }
#endif
        else if( no>= PETHD_START&&no<PETHD_END){
            return strerror(no-PETHD_START);
        }else{
            return (no>=0&&no<ARRAY_SIZE(g_err_reason))?g_err_reason[no]:NULL;
        }
        pset_errno(0);
}
const char* pstr_lasterr(void)
{
    return pstr_errno(pget_errno());
}