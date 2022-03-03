#ifndef __PLT_ASSERT___H__
#define __PLT_ASSERT___H__
#include<stdlib.h>
#include<assert.h>
#include<string.h>
#include"plog_plus.h"

#ifdef NDEBUG
#define plt_assert(p)\
do{\
    if(!(p)){\
            PLOG_FATAL("FATAL","(%s:%d)(%s) failed\n",basename(__FILE__),__LINE__,#p);\
            exit(1);\
        }\
}while(0)
#else
#define plt_assert assert
#endif

#endif