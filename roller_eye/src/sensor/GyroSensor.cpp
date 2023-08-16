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
#include <math.h>
#include <sys/select.h>
#include"roller_eye/plog_plus.h"
#include "l3g4200d.h"
#include"nusensors.h"
#include "GyroSensor.h"

#define GYRO_TAG    "gyro"
/*****************************************************************************/

GyroSensor::GyroSensor()
    : SensorBase(GY_DEVICE_NAME, "gyro"),
      mEnabled(0),
      mInputReader(32)
{
    memset(gyro_offset, 0, sizeof(gyro_offset));

    mPendingEvent.version = sizeof(sensors_event_t);
    mPendingEvent.sensor = ID_GY;
    mPendingEvent.type = SENSOR_TYPE_GYROSCOPE;
    mPendingEvent.gyro.status = SENSOR_STATUS_ACCURACY_HIGH;
    
    int err = 0;
    err = open_device();
    err = err<0 ? -errno : 0;
    if (err) {
        PLOG_ERROR(GYRO_TAG,"open device fail\n");
        return;
    }

    readCalibration();
}

GyroSensor::~GyroSensor() {
    if (mEnabled) {
        enable(0, 0);
    }
    if (dev_fd > 0) {
        close(dev_fd);
        dev_fd = -1;
    }
}

int GyroSensor::enable(int32_t, int en)
{
	int flags = en ? 1 : 0;
	int err = 0;

	if (flags != mEnabled) {
		if (dev_fd < 0) {
			open_device();
		}
		err = ioctl(dev_fd, L3G4200D_IOCTL_SET_ENABLE, &flags);
		err = err<0 ? -errno : 0;
        if(err){
            PLOG_ERROR(GYRO_TAG, "L3G4200D_IOCTL_SET_ENABLE failed");
        }
		
		if (!err) {
			mEnabled = en ? 1 : 0;
		}
	}
	return err;
}

int GyroSensor::setDelay(int32_t handle, int64_t ns)
{
    int result = 0;

    if (ns < 0)
        return -EINVAL;

    if (dev_fd < 0)
        open_device();

    int delay = ns / 1000000;
    PLOG_INFO(GYRO_TAG, "Gyrosensor update delay: %dms\n", delay);

    if (0 > (result = ioctl(dev_fd, L3G4200D_IOCTL_SET_DELAY, &delay))){
        PLOG_ERROR(GYRO_TAG, "fail to perform L3G4200D_IOCTL_SET_DELAY\n");}
    else{
        PLOG_INFO(GYRO_TAG, "update gyro delay to %d ms\n", delay);}

    return result;
}

int GyroSensor::isActivated(int /* handle */)
{
    return mEnabled;
}

int GyroSensor::readEvents(sensors_event_t* data, int count)
{
    if (count < 1)
        return -EINVAL;

    ssize_t n = mInputReader.fill(data_fd);
    if (n < 0)
        return n;

    int numEventReceived = 0;
    input_event const* event;

    while (count && mInputReader.readEvent(&event)) {
        int type = event->type;
        if (type == EV_REL) {
            processEvent(event->code, event->value);
        }else if (type == EV_SYN) {
            mPendingEvent.timestamp = timevalToNano(event->time);
            *data++ = mPendingEvent;
            count--;
            numEventReceived++;
        }else {
            PLOG_ERROR(GYRO_TAG, "GyroSensor: unknown event (type=%d, code=%d)", type, event->code);
        }
        mInputReader.next();
    }

    return numEventReceived;
}

void GyroSensor::processEvent(int code, int value)
{
    switch (code) {
        case EVENT_TYPE_GYRO_X:
            mPendingEvent.gyro.x = (value - gyro_offset[0]) * CONVERT_GYRO_X;
            break;
        case EVENT_TYPE_GYRO_Y:
            mPendingEvent.gyro.y = (value - gyro_offset[1]) * CONVERT_GYRO_Y;
            break;
        case EVENT_TYPE_GYRO_Z:
            mPendingEvent.gyro.z = (value - gyro_offset[2]) * CONVERT_GYRO_Z;
            break;
    }
}

void GyroSensor::readCalibration()
{
    if (dev_fd < 0)
        open_device();
    if (dev_fd<0){
        PLOG_ERROR(GYRO_TAG,"fail to perform GyroSensor open_device ");
        return;
    }

    // PLOG_ERROR(GYRO_TAG,"don't GyroSensor readCalibration!");
    // return;

    int result = ioctl(dev_fd, L3G4200D_IOCTL_GET_CALIBRATION, &gyro_offset);
    if (result < 0){
        PLOG_ERROR(GYRO_TAG,"fail to perform L3G4200D_IOCTL_GET_CALIBRATION");}
    else{
        PLOG_INFO(GYRO_TAG,"gyro calibration is %d, %d, %d\n", gyro_offset[0], gyro_offset[1], gyro_offset[2]);}
}
