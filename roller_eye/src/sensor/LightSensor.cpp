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
#include <sys/select.h>
#include"roller_eye/plog_plus.h"
#include"nusensors.h"
#include "isl29028.h"
#include "LightSensor.h"

#define GYRO_TAG    "light"
/*****************************************************************************/
LightSensor::LightSensor()
    : SensorBase(LS_DEVICE_NAME, "lightsensor-level"),
      mEnabled(0),
      mInputReader(4),
      mHasPendingEvent(false)
{
    mPendingEvent.version = sizeof(sensors_event_t);
    mPendingEvent.sensor = ID_L;
    mPendingEvent.type = SENSOR_TYPE_LIGHT;

    open_device();

    int flags = 0;
    if (dev_fd > 0) {
        if (!ioctl(dev_fd, LIGHTSENSOR_IOCTL_GET_ENABLED, &flags)) {
            PLOG_ERROR(GYRO_TAG,"LIGHTSENSOR_IOCTL_GET_ENABLED flags:%d\n", flags);
            if (flags) {
                mEnabled = 1;
                setInitialState();
            }
        }else{
            PLOG_ERROR(GYRO_TAG,"LIGHTSENSOR_IOCTL_GET_ENABLED flags:%d\n", flags);
        }
    }
}

LightSensor::~LightSensor() {
    if (mEnabled) {
        enable(0, 0);
    }
    if (dev_fd > 0) {
        close(dev_fd);
        dev_fd = -1;
    }
}

int LightSensor::setInitialState() {
    struct input_absinfo absinfo;
    if ((data_fd > 0) && (!ioctl(data_fd, EVIOCGABS(EVENT_TYPE_LIGHT), &absinfo))) {
		mPendingEvent.light = absinfo.value;// indexToValue(absinfo.value); //elvis modify
        if (mPendingEvent.light != mPreviousLight) {
        	mHasPendingEvent = true;
             mPreviousLight = mPendingEvent.light;
        }
    }
    return 0;
}

int LightSensor::setDelay(int32_t /* handle */, int64_t ns)
{
    short ms;
    int ret = -1;

    ms = ns / 1000000;

    if (dev_fd < 0) {
        open_device();
    }

    ret = ioctl(dev_fd, LIGHTSENSOR_IOCTL_SET_RATE, &ms);
    if (ret){
        PLOG_ERROR(GYRO_TAG,"LIGHTSENSOR_IOCTL_SET_RATE failed\n");
    }

    return ret;
}

int LightSensor::enable(int32_t, int en) {
    int flags = en ? 1 : 0;
    int err = 0;
    mPreviousLight = -1;

    if (flags != mEnabled) {
        if (dev_fd < 0) {
            open_device();
        }
        err = ioctl(dev_fd, LIGHTSENSOR_IOCTL_ENABLE, &flags);
        err = err<0 ? -errno : 0;
        if(err!=0){
            PLOG_ERROR(GYRO_TAG,"LIGHTSENSOR_IOCTL_ENABLE failed (%s)", strerror(-err));
        }
        PLOG_INFO(GYRO_TAG,"LightSensor::enable flags:%d\n", flags);
        if (!err) {
            mEnabled = en ? 1 : 0;
            if (en) {
                setInitialState();
            }
        }
    }
    return 0;
}

int LightSensor::isActivated(int /* handle */)
{
    return mEnabled;
}

bool LightSensor::hasPendingEvents() const {
    return mHasPendingEvent;
}

int LightSensor::readEvents(sensors_event_t* data, int count)
{
    if (count < 1){
        PLOG_ERROR(GYRO_TAG,"%s:%d", __FILE__,__LINE__);
        return -EINVAL;
    }

    if (mHasPendingEvent) {
        mHasPendingEvent = false;
        mPendingEvent.timestamp = getTimestamp();
        *data = mPendingEvent;
        return mEnabled ? 1 : 0;
    }

    ssize_t n = mInputReader.fill(data_fd);
    //PLOG_ERROR(GYRO_TAG,"LightSensor: readEvents fill return %d",n);
    if (n < 0){
        return n;
    }

    int numEventReceived = 0;
    input_event const* event;

    while (count && mInputReader.readEvent(&event)) {
        int type = event->type;
        if (type == EV_ABS) {
            if (event->code == EVENT_TYPE_LIGHT) {
                //PLOG_ERROR(GYRO_TAG,"LightSensor: readEvents return %d",event->value );
                if (event->value != -1) {
                    // FIXME: not sure why we're getting -1 sometimes
					mPendingEvent.light = event->value; //indexToValue(event->value); //elvis modify
                }
            }
        } else if (type == EV_SYN) {
            mPendingEvent.timestamp = getTimestamp();
            if (mEnabled /*&& (mPendingEvent.light != mPreviousLight)*/) {
                int ch0 = ((int)mPendingEvent.light >>16) & 0xFFFF;
                int ch1 = (int)mPendingEvent.light & 0xFFFF;
                //PLOG_ERROR(GYRO_TAG,"LightSensor: readEvents ch0=%d , ch1=%d", ch0, ch1);
                *data++ = mPendingEvent;
                count--;
                numEventReceived++;
                mPreviousLight = mPendingEvent.light;
            }
        } else {
            PLOG_ERROR(GYRO_TAG,"LightSensor: unknown event (type=%d, code=%d)",
                    type, event->code);
        }
        mInputReader.next();
    }

    return numEventReceived;
}

float LightSensor::indexToValue(size_t index) const
{
    static const float luxValues[8] = {
            10.0, 160.0, 225.0, 320.0,
            640.0, 1280.0, 2600.0, 10240.0
    };
	//elvis modify
    //const size_t maxIndex = sizeof(luxValues)/sizeof(*luxValues) - 1;
    //if (index > maxIndex)
     //   index = maxIndex;
	return index;// luxValues[index];
}
