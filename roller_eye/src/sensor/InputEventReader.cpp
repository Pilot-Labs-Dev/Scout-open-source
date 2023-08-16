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

#include <stdint.h>
#include <errno.h>
#include <unistd.h>
#include <poll.h>
#include <string.h>

#include <sys/cdefs.h>
#include <sys/types.h>

#include <linux/input.h>

#include "roller_eye/plog_plus.h"

#include "InputEventReader.h"

#define INPUT_READER_TAG "InputReader"
/*****************************************************************************/

struct input_event;

InputEventCircularReader::InputEventCircularReader(size_t numEvents)
    : mBuffer(new input_event[numEvents * 2]),

      mBufferEnd(mBuffer + numEvents),
      mHead(mBuffer),
      mCurr(mBuffer),
      mFreeSpace(numEvents)
{
    PLOG_DEBUG(INPUT_READER_TAG,"Entered : numEvents = %d.", (int)numEvents);
}

InputEventCircularReader::~InputEventCircularReader()
{
    delete [] mBuffer;
}

ssize_t InputEventCircularReader::fill(int fd)
{
    size_t numEventsRead = 0;
    if (mFreeSpace) {
        const ssize_t nread = read(fd, mHead, mFreeSpace * sizeof(input_event));
        if (nread<0 || nread % sizeof(input_event)) {
            // we got a partial event!!
            return nread<0 ? -errno : -EINVAL;
        }

        numEventsRead = nread / sizeof(input_event);
        //dumpEvents(mHead, numEventsRead);
        //PLOG_INFO(INPUT_READER_TAG,"nread = %d, numEventsRead = %d.", (int)nread, (int)numEventsRead);
        if (numEventsRead) {

            input_event *event = (input_event*)mHead;
            //dumpEvents(event, numEventsRead);
            // PLOG_ERROR(INPUT_READER_TAG,"InputEventCircularReader::fill type=%d, code=%d, value=%d", event->type,
            //     event->code, event->value);

            mHead += numEventsRead;
            mFreeSpace -= numEventsRead;
            if (mHead > mBufferEnd) {
                size_t s = mHead - mBufferEnd;
                memcpy(mBuffer, mBufferEnd, s * sizeof(input_event));
                mHead = mBuffer + s;
            }
        }
    }

    return numEventsRead;
}

ssize_t InputEventCircularReader::readEvent(input_event const** events)
{
    *events = mCurr;
    ssize_t available = (mBufferEnd - mBuffer) - mFreeSpace;
    return available ? 1 : 0;
}

void InputEventCircularReader::next()
{
    mCurr++;
    mFreeSpace++;
    if (mCurr >= mBufferEnd) {
        mCurr = mBuffer;
    }
}

void InputEventCircularReader::dumpEvents(input_event const * events, int eventsNum)
{
    PLOG_DEBUG(INPUT_READER_TAG,"to dump %d events :", eventsNum);
    int i = 0;
    input_event* current = NULL;
    for ( i = 0; i < eventsNum; i++ )
    {
        current = (input_event*)events + i;
        int ch0 = (current->value >> 16) & 0xFFFF;
        int ch1 = current->value & 0xFFFF;
        PLOG_DEBUG(INPUT_READER_TAG,"event '%d' : type : 0x%x, code : 0x%x ch0:%d ch1:%d.",
                 i, current->type, current->code, ch0, ch1);
    }
};

