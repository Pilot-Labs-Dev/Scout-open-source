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
#include <roller_eye/plog_plus.h>

#include "isl29028.h"
#include"nusensors.h"
#include "ProximitySensor.h"

#define PROXIMITY_TAG   "TOF"
/*****************************************************************************/

ProximitySensor::ProximitySensor()
    : SensorBase(PS_DEVICE_NAME, "proximity"),
      mEnabled(0),
      mInputReader(4),
      mHasPendingEvent(false)
{
    mPendingEvent.version = sizeof(sensors_event_t);
    mPendingEvent.sensor = ID_P;
    mPendingEvent.type = SENSOR_TYPE_PROXIMITY;
    
    open_device();

    int flags = 0;
    if (dev_fd > 0) {
        if (!ioctl(dev_fd, PSENSOR_IOCTL_GET_ENABLED, &flags)) {
            if (flags) {
                mEnabled = 1;
                setInitialState();
            }
        }
    }
}

ProximitySensor::~ProximitySensor() {
    if (dev_fd > 0) {
        close(dev_fd);
        dev_fd = -1;
    }
}

int ProximitySensor::setInitialState() {
    struct input_absinfo absinfo;
    if ((data_fd > 0) && (!ioctl(data_fd, EVIOCGABS(EVENT_TYPE_PROXIMITY), &absinfo))) {
        // make sure to report an event immediately
        mHasPendingEvent = true;
        mPendingEvent.distance = indexToValue(absinfo.value);
    }
    return 0;
}

int ProximitySensor::enable(int32_t, int en) {
    int newState = en ? 1 : 0;
    int err = 0;
    if (newState != mEnabled) {
        if (dev_fd < 0) {
            open_device();
        }
        int flags = newState;
        err = ioctl(dev_fd, PSENSOR_IOCTL_ENABLE, &flags);
        err = err<0 ? -errno : 0;
        if(err!=0){
            PLOG_ERROR(PROXIMITY_TAG, "PSENSOR_IOCTL_ENABLE failed\n");
        }
        if (!err) {
            mEnabled = newState;
            if (en) {
                setInitialState();
            }
        }
    }
    return err;
}

bool ProximitySensor::hasPendingEvents() const {
    return mHasPendingEvent;
}

int ProximitySensor::isActivated(int /* handle */)
{
    return mEnabled;
}

int ProximitySensor::readEvents(sensors_event_t* data, int count)
{
    if (count < 1)
        return -EINVAL;

    if (mHasPendingEvent) {
        mHasPendingEvent = false;
        mPendingEvent.timestamp = getTimestamp();
        *data = mPendingEvent;
        return mEnabled ? 1 : 0;
    }

    ssize_t n = mInputReader.fill(data_fd);
    if (n < 0){
        return n;
    }

    int numEventReceived = 0;
    input_event const* event;

    while (count && mInputReader.readEvent(&event)) {
        int type = event->type;
        if (type == EV_ABS) {
            if (event->code == EVENT_TYPE_PROXIMITY) {
                mPendingEvent.distance = indexToValue(event->value);
            }
        } else if (type == EV_SYN) {
            mPendingEvent.timestamp = timevalToNano(event->time);
            if (mEnabled) {
                *data++ = mPendingEvent;
                count--;
                numEventReceived++;
            }
        } else {
            PLOG_ERROR(PROXIMITY_TAG, "ProximitySensor: unknown event (type=%d, code=%d)\n",
                    type, event->code);
        }
        mInputReader.next();
    }
    return numEventReceived;
}

float ProximitySensor::indexToValue(size_t index) const
{
    return index * PROXIMITY_THRESHOLD_CM;
}
