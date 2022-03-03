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

#ifndef ANDROID_SENSOR_BASE_H
#define ANDROID_SENSOR_BASE_H
#define INSERT_FAKE_MAX 5
#define INSERT_DUR_MAX 8
#define INSERT_DUR_MIN 5

#include <stdint.h>
#include <errno.h>
#include <string.h>
#include <sys/cdefs.h>
#include <sys/types.h>
#include<sys/time.h>


/*****************************************************************************/

struct vec3_data_t{
    int status;
    double x;
    double y;
    double z;
};
struct sensors_event_t{
    int version;
    int sensor;
    int type;
    int64_t timestamp;
    union{
        vec3_data_t gyro;
        vec3_data_t acceleration;
        vec3_data_t magnetic;
        double distance;
        double light;
    };
};

class SensorBase {
protected:
    const char* dev_name;
    const char* data_name;
    int         dev_fd;
    int         data_fd;

    static int openInput(const char* inputName);
    static int64_t getTimestamp();
    static int64_t timevalToNano(timeval const& t) {
        return t.tv_sec*1000000000LL + t.tv_usec*1000;
    }

    int open_device();
    int close_device();

public:
            SensorBase(
                    const char* dev_name,
                    const char* data_name);

    virtual ~SensorBase();

    virtual int readEvents(sensors_event_t* data, int count) = 0;
    virtual bool hasPendingEvents() const;
    virtual int getFd() const;
    virtual int setDelay(int32_t handle, int64_t ns);
    virtual int enable(int32_t handle, int enabled) = 0;
    virtual int isActivated(int handle);
};

int sensor_open_input(const char* name);
/*****************************************************************************/

#endif  // ANDROID_SENSOR_BASE_H
