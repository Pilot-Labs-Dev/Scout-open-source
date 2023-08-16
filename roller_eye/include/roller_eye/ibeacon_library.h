#ifndef CSI_IBEACON_ROSCPP_LIBRARY_H
#define CSI_IBEACON_ROSCPP_LIBRARY_H

#include <ros/ros.h>

int ibeacon_start();
int ibeacon_stop();
float ibeacon_range(std::string uuid,std::string major,std::string minor,float n = 3);

#endif
