/*
 * Copyright (C) 2016 Rockchip Electronics Co.Ltd
 * Authors:
 *	Zhiqin Wei <wzq@rock-chips.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <stdint.h>
//#include <vector>
#include <sys/types.h>

//////////////////////////////////////////////////////////////////////////////////
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <unistd.h>

#include <sys/mman.h>
#include <linux/stddef.h>

#include "stdio.h"

#include "drmrga.h"

/* flip source image horizontally (around the vertical axis) */                                                                                                     
#define HAL_TRANSFORM_FLIP_H     0x01
/* flip source image vertically (around the horizontal axis)*/                                                                                                      
#define HAL_TRANSFORM_FLIP_V     0x02
/* rotate source image 90 degrees clockwise */                                                                                                                      
#define HAL_TRANSFORM_ROT_90     0x04
/* rotate source image 180 degrees */                                                                                                                               
#define HAL_TRANSFORM_ROT_180    0x03
/* rotate source image 270 degrees clockwise */                                                                                                                     
#define HAL_TRANSFORM_ROT_270    0x07


//////////////////////////////////////////////////////////////////////////////////

// -------------------------------------------------------------------------------

class RockchipRga 
{
/************************************public**************************************/

public:

    RockchipRga();
    ~RockchipRga();

    int         RkRgaInit();
    int         RkRgaGetAllocBuffer(bo_t *bo_info, int width, int height, int bpp);
    int         RkRgaGetMmap(bo_t *bo_info);
    int         RkRgaUnmap(bo_t *bo_info);
    int         RkRgaFree(bo_t *bo_info);
    int         RkRgaGetBufferFd(bo_t *bo_info, int *fd);
    int         RkRgaBlit(rga_info *src, rga_info *dst, rga_info *src1);
    int         RkRgaCollorFill(rga_info *dst);


    void        RkRgaSetLogOnceFlag(int log) {mLogOnce = log;}
    void        RkRgaSetAlwaysLogFlag(bool log) {mLogAlways = log;}
    void        RkRgaLogOutRgaReq(struct rga_req rgaReg);
    int         RkRgaLogOutUserPara(rga_info *rgaInfo);
    inline bool RkRgaIsReady() { return mSupportRga; }

/************************************private***********************************/
private:
    bool                            mSupportRga;
    int                             mLogOnce;
    int                             mLogAlways;
    void *                          mContext;
};

// ---------------------------------------------------------------------------


