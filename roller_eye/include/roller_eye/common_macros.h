#ifndef __COMMON_MACROS___H__
#define __COMMON_MACROS___H__
#include<stdlib.h>
#include"platfarm_define.h"

#ifdef __cplusplus
extern "C"{
#endif

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(a)   (sizeof(a)/sizeof((a)[0]))
#endif

#ifndef MAX
#define MAX(a,b)   ((a)>(b)?(a):(b))
#endif

#ifndef MIN
#define MIN(a,b)   ((a)<(b)?(a):(b))
#endif

#ifdef __cplusplus
}
#endif
#endif