#ifndef __PLT_MALLOC___H__
#define __PLT_MALLOC___H__
#include<stddef.h>
#ifdef __cplusplus
extern "C"{
#endif

#define PMALLOC_FLAG_FAIL_LOG                                 0x00000001
#define PMALLOC_FLAG_FAIL_EXIT                                0x00000002


void* __plt_malloc(size_t size,const char *func,int line,int flag);
void* __plt_calloc(size_t n,size_t size,const char *func,int line,int flag);
void *__plt_realloc(void *ptr, size_t size,const char *func,int line,int flag);
void *__plt_reallocarray(void *ptr, size_t n, size_t size,const char *func,int line,int flag);
void plt_free(void *ptr);

#define _plt_malloc(size,flag) __plt_malloc(size,__FUNCTION__,__LINE__,flag)
#define _plt_calloc(n,size,flag) __plt_calloc(n,size,__FUNCTION__,__LINE__,flag)
#define _plt_realloc(ptr,size,flag) __plt_realloc(ptr,size,__FUNCTION__,__LINE__,flag)
#define _plt_reallocarray(ptr,n,size,flag) __plt_reallocarray(ptr,n,size,__FUNCTION__,__LINE__,flag)  

#define plt_malloc(size) _plt_malloc(size,0)
#define plt_calloc(n,size)_plt_calloc(n,size,0)
#define plt_realloc(size) _plt_realloc(size,0)
#define plt_reallocarray(n,size) _plt_reallocarray(,n,size,0)

#define plt_malloc_exit(size) _plt_malloc(size,PMALLOC_FLAG_FAIL_EXIT)
#define plt_calloc_exit(n,size)_plt_calloc(n,size,PMALLOC_FLAG_FAIL_EXIT)
#define plt_realloc_exit(size) _plt_realloc(size,PMALLOC_FLAG_FAIL_EXIT)
#define plt_reallocarray_exit(n,size) _plt_reallocarray(,n,size,PMALLOC_FLAG_FAIL_EXIT)

#define plt_malloc_log(size) _plt_malloc(size,PMALLOC_FLAG_FAIL_LOG)
#define plt_calloc_log(n,size)_plt_calloc(n,size,PMALLOC_FLAG_FAIL_LOG)
#define plt_realloc_log(size) _plt_realloc(size,PMALLOC_FLAG_FAIL_LOG)
#define plt_reallocarray_log(n,size) _plt_reallocarray(,n,size,PMALLOC_FLAG_FAIL_LOG)


#ifdef __cplusplus
}
#endif
#endif