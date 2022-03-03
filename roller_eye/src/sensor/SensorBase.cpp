/*
 * Copyright (C) 2008 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <unistd.h>
#include <dirent.h>
#include<time.h>

#include <linux/input.h>

#include "SensorBase.h"
#include"roller_eye/plog_plus.h"

//#define ENABLE_DEBUG_LOG
#define SENSOR_TAG  "Sensor"

/*****************************************************************************/

SensorBase::SensorBase(
        const char* dev_name,
        const char* data_name)
    : dev_name(dev_name), data_name(data_name),
      dev_fd(-1), data_fd(-1)
{
    data_fd = openInput(data_name);
}

SensorBase::~SensorBase() {
    if (data_fd >= 0) {
        close(data_fd);
        data_fd = -1;
    }
    if (dev_fd >= 0) {
        close(dev_fd);
        dev_fd = -1;
    }
}

int SensorBase::open_device() {
    if (dev_fd<0 && dev_name) {
        dev_fd = open(dev_name, O_RDONLY);
        if (dev_fd<0) {
            return -errno;
        }        
    }
    return 0;
}

int SensorBase::close_device() {
    if (dev_fd >= 0) {
        close(dev_fd);
        dev_fd = -1;
    }
    return 0;
}

int SensorBase::getFd() const {
    return data_fd;
}

int SensorBase::setDelay(int32_t handle, int64_t ns) {
    return 0;
}

bool SensorBase::hasPendingEvents() const {
    return false;
}

int64_t SensorBase::getTimestamp() {
    // struct timespec t;
    // t.tv_sec = t.tv_nsec = 0;
    // clock_gettime(CLOCK_MONOTONIC, &t);
    // return int64_t(t.tv_sec)*1000000000LL + t.tv_nsec;
    return 0;
}

struct input_dev {
    int fd;
    char name[80];
};

static int getInput(const char *inputName)
{
    int fd = -1;
    unsigned i;
    static bool first = true;
    static struct input_dev dev[255];

    if (first) {
        int fd = -1;
        const char *dirname = "/dev/input";
        char devname[PATH_MAX];
        char *filename;
        DIR *dir;
        struct dirent *de;

        first = false;
        for (i = 0; i < sizeof(dev)/sizeof(dev[0]); i++) {
            dev[i].fd = -1;
            dev[i].name[0] = '\0';
        }
        i = 0;

        dir = opendir(dirname);
        if (dir == NULL)
            return -1;
        strcpy(devname, dirname);
        filename = devname + strlen(devname);
        *filename++ = '/';
        while ((de = readdir(dir))) {
            if (de->d_name[0] == '.' &&
                (de->d_name[1] == '\0' || (de->d_name[1] == '.' && de->d_name[2] == '\0')))
                    continue;
            strcpy(filename, de->d_name);
            fd = open(devname, O_RDONLY);
            if (fd >= 0) {
                char name[80];
                if (ioctl(fd, EVIOCGNAME(sizeof(name) - 1), &name) >= 1) {
                    dev[i].fd = fd;
                    strncpy(dev[i].name, name, sizeof(dev[i].name));
                }
            }
            i++;
        }
        closedir(dir);
    }

    for (i = 0; i < sizeof(dev)/sizeof(dev[0]); i++) {
        if (!strncmp(inputName, dev[i].name, sizeof(dev[i].name))) {
            fd = dev[i].fd;
            break;
        }
    }
    if(fd<0){
        PLOG_WARN(SENSOR_TAG,"couldn't find '%s' input device", inputName);
    }

    return fd;
}

int SensorBase::openInput(const char* inputName) {
    return getInput(inputName);
}

int SensorBase::enable(int32_t /* handle */, int /* enabled */)
{
    return 0;
}

int SensorBase::isActivated(int /* handle */)
{
    return 0;
}

int sensor_open_input(const char* name)
{
    return getInput(name);
}
