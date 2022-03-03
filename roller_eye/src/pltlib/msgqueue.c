#include<stdlib.h>
#include<pthread.h>
#include<unistd.h>
#include<sys/select.h>
#include<sys/time.h>
#include"msgqueue.h"
#include"plterrno.h"
#include"common_macros.h"

static const int usPerSec=1000000;

struct pqueue{
    int pipe[2];
    int rdIdx;
    int wrIdx;
    int cnt;
    int max;
    queue_proc proc;
    struct pqueue_msg *msg;
    pthread_mutex_t mutex;
    pthread_cond_t wr;
};
QHandle pqueue_create(int max,queue_proc proc)
{
    int ret;
    struct pqueue *q=calloc(1,sizeof(struct pqueue));
    if(q==NULL){
        pset_errno(PEMEM);
        return NULL;
    }

    if((q->msg=calloc(max,sizeof(struct pqueue_msg)))==NULL){
        pset_errno(PEMEM);
        goto out0;
    }
    
    if((ret=pthread_mutex_init(&q->mutex,NULL))!=0){
        pset_pthreadno(ret);
        goto out1;
    }
    if((ret=pthread_cond_init(&q->wr,NULL))!=0){
        pset_pthreadno(ret);
        goto out2;
    }
   
    if(pipe(q->pipe)<0){
            pset_errno(PESYS);
            goto out3;
    }
    q->proc=proc;
    q->max=max;
    return q;
out3:
    pthread_cond_destroy(&q->wr);
out2:
    pthread_mutex_destroy(&q->mutex);
out1:
    free(q->msg);
out0:
    free(q);
    return NULL;
}
#define ASSER_HANDLE(handle) \
    struct pqueue *q=(struct pqueue*)handle;\
    if(q==NULL){\
        pset_errno(PENULL);\
        return -1;\
    }

int pqueue_destroy(QHandle handle)
{
    ASSER_HANDLE(handle);
    
    if(q->pipe[0]<0){
        pset_errno(PEBUG);
        return -1;
    }
    close(q->pipe[0]);
    close(q->pipe[1]);
    q->pipe[0]=q->pipe[1]=-1;
    
    pthread_cond_destroy(&q->wr);
    pthread_mutex_destroy(&q->mutex);
    free(q->msg);
    free(q);
    return 0;
}
int pqueue_push(QHandle handle,struct pqueue_msg msg,int wait)
{
    int ret=-1,code;
    ASSER_HANDLE(handle);
    if((code=pthread_mutex_lock(&q->mutex))!=0){
        pset_pthreadno(code);
        return -1;
    }

    while(q->cnt==q->max){
        if(q->rdIdx!=q->wrIdx){
            pset_errno(PEBUG);
            goto out;
        }
        if(!wait){
            pset_errno(PEFULL);
            goto out;
        }else{
            if((code=pthread_cond_wait(&q->wr,&q->mutex))!=0){
                pset_pthreadno(code);
                goto out;
            }
        }
    }
    if(write(q->pipe[1],"r",1)!=1){
        pset_errno(PESYS);
        goto out;
    }
    q->msg[q->wrIdx]=msg;
    if(q->wrIdx==q->max-1){
        q->wrIdx=0;
    }else{
        q->wrIdx++;
    }
    q->cnt++;
    ret=0;
out:
    pthread_mutex_unlock(&q->mutex);
    return ret;
}
int pqueue_pop(QHandle handle)
{
    int ret=-1,code,msgvalid=0;
    char t;
    ASSER_HANDLE(handle);
    struct pqueue_msg msg;

    if((code=pthread_mutex_lock(&q->mutex))!=0){
        pset_pthreadno(code);
        return -1;
    }
    if(q->cnt==0)
    {
        pset_errno(q->rdIdx==q->wrIdx?PEEMPTY:PEBUG);
        goto out;
    }
    if(read(q->pipe[0],&t,1)!=1){
        pset_errno(PESYS);
        goto out;
    }
    msg=q->msg[q->rdIdx];
    if(q->rdIdx==q->max-1){
        q->rdIdx=0;
    }else{
        q->rdIdx++;
    }
    q->cnt--;
    ret=0;
    msgvalid=1;
    if(q->cnt==q->max-1){
        if((code=pthread_cond_signal(&q->wr))!=0){
            pset_pthreadno(code);
            ret=-1;
        }
    }
out:
    pthread_mutex_unlock(&q->mutex);
    if(msgvalid){
        q->proc(&msg);
    }
    return ret;
}
static int wait_read(int fd,int timeout)
{
    fd_set rd;
    struct timeval val;
    struct timeval *pval=NULL;

    FD_ZERO(&rd);
    FD_SET(fd,&rd);

    if(timeout>0){
        val.tv_sec=timeout/usPerSec;
        val.tv_usec=timeout%usPerSec;
        pval=&val;
    }

    if(select(fd+1,&rd,NULL,NULL,pval)<0){
        pset_errno(PESYS);
        return -1;
    }
    return 0;
}
int pqueue_pop_timeout(QHandle handle,int timeout)
{
    if(wait_read(pqueue_get_notify_fd(handle),timeout)<0){
        return -1;
    }
    return pqueue_pop(handle);
}
int pqueue_get_notify_fd(QHandle handle)
{
    ASSER_HANDLE(handle);
    return q->pipe[0];
}