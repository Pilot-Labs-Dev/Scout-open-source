#ifndef __MSG_QUEUE__H___
#define __MSG_QUEUE__H___
#ifdef __cplusplus
extern "C"{
#endif

struct pqueue_msg{
    int msg;
    int pars;
    void* parc;
};

typedef void* QHandle;
typedef void (*queue_proc)(struct pqueue_msg *msg );


QHandle pqueue_create(int max,queue_proc proc);

int pqueue_destroy(QHandle handle);

int pqueue_push(QHandle handle,struct pqueue_msg msg,int wait);

int pqueue_pop(QHandle handle);//noblock

int pqueue_pop_timeout(QHandle handle,int timeout);//block

int pqueue_get_notify_fd(QHandle handle);

#ifdef __cplusplus
}
#endif
#endif