/***
  This file is part of PulseAudio.

  PulseAudio is free software; you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as published
  by the Free Software Foundation; either version 2.1 of the License,
  or (at your option) any later version.

  PulseAudio is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
  General Public License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with PulseAudio; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307
  USA.
***/

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>

#include <pulse/simple.h>
#include <pulse/error.h>
#include <pulse/gccmacro.h>

#include "roller_eye/plog_plus.h"

#define BUFSIZE 1024

int testAudio(int argc, char*argv[]) {

    /* The Sample format to use */
    static const pa_sample_spec ss = {
        .format = PA_SAMPLE_S16LE,
        .rate = 16000/2,
        .channels = 2
    };

    pa_simple *s = NULL;
    int ret = 1;
    int error;

    /* replace STDIN with the specified file if needed */

    int fd;

    PLOG_DEBUG(NAV_PATH_NODE_TAG, ": testAudio\n");
    if ((fd = open("/mnt/audio/network_break.wav", O_RDONLY)) < 0) {
        fprintf(stderr, __FILE__": open() failed: %s\n", strerror(errno));
        PLOG_DEBUG(NAV_PATH_NODE_TAG, ": open() failed: %s\n", strerror(errno));
        goto finish;
    }

    if (dup2(fd, STDIN_FILENO) < 0) {
        fprintf(stderr, __FILE__": dup2() failed: %s\n", strerror(errno));
        PLOG_DEBUG(NAV_PATH_NODE_TAG, ":  dup2() failed: %s\n", strerror(errno));
        goto finish;
    }

    close(fd);


    /* Create a new playback stream */
    if (!(s = pa_simple_new(NULL, "tutkPlayer", PA_STREAM_PLAYBACK, NULL, "playback", &ss, NULL, NULL, &error))) {
        PLOG_DEBUG(NAV_PATH_NODE_TAG, ":  pa_simple_new\n");
        goto finish;
    }

    for (;;) {
        uint8_t buf[BUFSIZE];
        ssize_t r;

        /* Read some data ... */
        if ((r = read(STDIN_FILENO, buf, sizeof(buf))) <= 0) {
            if (r == 0) /* EOF */
                break;

            fprintf(stderr, __FILE__": read() failed: %s\n", strerror(errno));
            PLOG_DEBUG(NAV_PATH_NODE_TAG, ": read() failed: %s\n", strerror(errno));
            goto finish;
        }

        /* ... and play it */
        if (pa_simple_write(s, buf, (size_t) r, &error) < 0) {
            //fprintf(stderr, __FILE__": pa_simple_write() failed: %s\n", pa_strerror(error));
            PLOG_DEBUG(NAV_PATH_NODE_TAG, ": pa_simple_write() failed: %s\n", strerror(error));
            goto finish;
        }
    }

    /* Make sure that every single sample was played */
    if (pa_simple_drain(s, &error) < 0) {
         // fprintf(stderr, __FILE__": pa_simple_drain() failed: %s\n", pa_strerror(error));
        PLOG_DEBUG(NAV_PATH_NODE_TAG, ": pa_simple_drain() failed %s\n", strerror(error));
        goto finish;
    }

    ret = 0;

finish:

    if (s)
        pa_simple_free(s);

    return ret;
}