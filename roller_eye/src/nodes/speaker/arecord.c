/*
 *  arecord.c - records
 *
 *      CREATIVE LABS CHANNEL-files
 *      Microsoft WAVE-files
 *      SPARC AUDIO .AU-files
 *      Raw Data
 *
 *  Copyright (c) by Jaroslav Kysela <perex@perex.cz>
 *  Based on vplay program by Michael Beck
 *
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#define _GNU_SOURCE
#include "aconfig.h"
#include <stdio.h>
#include <malloc.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <fcntl.h>
#include <ctype.h>
#include <errno.h>
#include <limits.h>
#include <time.h>
#include <locale.h>
#include <assert.h>
#include <termios.h>
#include <signal.h>
#include <poll.h>
#include <sys/uio.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <endian.h>
#include "gettext.h"
#include "formats.h"
#include "version.h"
#include "roller_eye/arecord.h"

#if 1
#define ABS(a)  (a) < 0 ? -(a) : (a)

#ifdef SND_CHMAP_API_VERSION
#define CONFIG_SUPPORT_CHMAP	1
#endif

#ifndef LLONG_MAX
#define LLONG_MAX    9223372036854775807LL
#endif

#ifndef le16toh
#include <asm/byteorder.h>
#define le16toh(x) __le16_to_cpu(x)
#define be16toh(x) __be16_to_cpu(x)
#define le32toh(x) __le32_to_cpu(x)
#define be32toh(x) __be32_to_cpu(x)
#endif

#define DEFAULT_FORMAT		SND_PCM_FORMAT_U8
#define DEFAULT_SPEED 		8000

#define FORMAT_DEFAULT		-1
#define FORMAT_RAW		0
#define FORMAT_VOC		1
#define FORMAT_WAVE		2
#define FORMAT_AU		3

/* global data */

static snd_pcm_sframes_t (*readi_func)(snd_pcm_t *s_handle, void *buffer, snd_pcm_uframes_t size);
static snd_pcm_sframes_t (*readn_func)(snd_pcm_t *s_handle, void **bufs, snd_pcm_uframes_t size);
typedef ssize_t (*xwrite_func)(int fd, const void *buf, size_t count);

static ssize_t xwrite(int fd, const void *buf, size_t count);

enum {
	VUMETER_NONE,
	VUMETER_MONO,
	VUMETER_STEREO
};

static char *s_command;
static snd_pcm_t *s_handle;
static struct {
	snd_pcm_format_t format;
	unsigned int channels;
	unsigned int rate;
} s_hwparams, s_rs_hwparams;
static int s_timelimit = 0;
static int s_sampleslimit = 0;
static int s_quiet_mode = 0;
static int s_file_type = FORMAT_DEFAULT;
static int s_open_mode = 0;
static snd_pcm_stream_t stream = SND_PCM_STREAM_CAPTURE;
static int s_mmap_flag = 0;
static int s_interleaved = 1;
static int s_nonblock = 0;
static volatile sig_atomic_t s_in_aborting = 0;
static snd_pcm_uframes_t s_chunk_size = 0;
static unsigned s_period_time = 0;
static unsigned s_buffer_time = 0;
static snd_pcm_uframes_t s_period_frames = 0;
static snd_pcm_uframes_t s_buffer_frames = 0;
static int s_avail_min = -1;
static int s_start_delay = 0;
static int s_stop_delay = 0;
static int s_monotonic = 0;
static int s_interactive = 0;
static int s_can_pause = 0;
static int s_fatal_errors = 0;
static int s_verbose = 0;
static int s_vumeter = VUMETER_NONE;
static size_t s_significant_bits_per_sample, s_bits_per_sample, s_bits_per_frame;
static size_t s_chunk_bytes;
static int s_test_nowait = 0;
static snd_output_t *s_log;
static long long s_max_file_size = 0;
static int s_max_file_time = 0;
static int s_use_strftime = 0;
volatile static int s_recycle_capture_file = 0;

static int s_fd = -1;
static off64_t s_pbrec_count = LLONG_MAX, s_fdcount;

/* needed prototypes */


static void capture(char *filename, xwrite_func callback);

static void suspend(void);
static int set_params(void);
static int safe_open(const char *name);

static const struct fmt_capture {
	void (*start) (int fd, size_t count);
	void (*end) (int fd);
	char *what;
	long long max_filesize;
} fmt_rec_table[] = {
	{	NULL,		NULL,		N_("raw data"),		LLONG_MAX }
};

#if __GNUC__ > 2 || (__GNUC__ == 2 && __GNUC_MINOR__ >= 95)
#define error(...) do {\
	fprintf(stderr, "%s: %s:%d: ", s_command, __func__, __LINE__); \
	fprintf(stderr, __VA_ARGS__); \
	putc('\n', stderr); \
} while (0)
#else
#define error(args...) do {\
	fprintf(stderr, "%s: %s:%d: ", s_command, __func__, __LINE__); \
	fprintf(stderr, ##args); \
	putc('\n', stderr); \
} while (0)
#endif

/*
 *	Subroutine to clean up before exit.
 */
static int prg_exit(int code)
{
	if (s_handle)
		snd_pcm_close(s_handle);

	return code;
}

static void signal_s_handler(int sig)
{
	if (s_in_aborting)
		return;

	s_in_aborting = 1;
	if (s_verbose==2)
		putchar('\n');
	if (!s_quiet_mode)
		fprintf(stderr, _("Aborted by signal %s...\n"), strsignal(sig));
	if (s_handle)
		snd_pcm_abort(s_handle);
	if (sig == SIGABRT) {
		/* do not call snd_pcm_close() and abort immediately */
		s_handle = NULL;
		prg_exit(EXIT_FAILURE);
	}
	signal(sig, SIG_DFL);
}

/* call on SIGUSR1 signal. */
static void signal_s_handler_recycle (int sig)
{
	/* flag the capture loop to start a new output file */
	s_recycle_capture_file = 1;
}

// /*
//  * make sure we write all bytes or return an error
//  */
// static ssize_t xwrite(int fd, const void *buf, size_t count)
// {
// 	return count;
// }

/*
 * make sure we write all bytes or return an error
 */
static ssize_t xwrite(int fd, const void *buf, size_t count)
{
	ssize_t written;
	size_t offset = 0;

	while (offset < count) {
		written = write(fd, (char *)buf + offset, count - offset);
		if (written <= 0)
			return written;

		offset += written;
	};

	return offset;
}

int arecord_init(snd_pcm_format_t format,unsigned int channels,unsigned int rate)
{
    //char *pcm_name = "default";
    char *pcm_name = "1mic_loopback";
    // char *pcm_name = "hw:0,1";
	int tmp, err, c;
	snd_pcm_info_t *info;

   s_timelimit = 60;   //add by ltl 2021-02-25
	snd_pcm_info_alloca(&info);
	err = snd_output_stdio_attach(&s_log, stderr, 0);
	s_chunk_size = -1;

	stream = SND_PCM_STREAM_CAPTURE;
	s_command = "arecord";
	s_start_delay = 1;
	s_file_type = FORMAT_RAW;

	s_rs_hwparams.channels = channels;
	s_rs_hwparams.rate = rate;
	s_rs_hwparams.format = format;
	//pcm_name = "hw:0,0";
	//s_buffer_frames = 2048;
	s_buffer_frames = 4096;

	err = snd_pcm_open(&s_handle, pcm_name, stream, s_open_mode);
	if (err < 0) {
		printf("audio open error: %s", snd_strerror(err));
		return 1;
	}

	if ((err = snd_pcm_info(s_handle, info)) < 0) {
		printf("info error: %s", snd_strerror(err));
		return 1;
	}

	if (s_nonblock) {
		err = snd_pcm_nonblock(s_handle, 1);
		if (err < 0) {
			printf("s_nonblock setting error: %s", snd_strerror(err));
			return 1;
		}
	}

	s_chunk_size = 1024;
	s_hwparams = s_rs_hwparams;

	readi_func = snd_pcm_readi;
	readn_func = snd_pcm_readn;

	signal(SIGINT, signal_s_handler);
	signal(SIGTERM, signal_s_handler);
	signal(SIGABRT, signal_s_handler);
	signal(SIGUSR1, signal_s_handler_recycle);

	return set_params();
	//capture("/mnt/audio/test2.pcm", xwrite);
}

void arecord_uninit()
{
    snd_pcm_close(s_handle);
	s_handle = NULL;
	snd_output_close(s_log);
	snd_config_update_free_global();
	prg_exit(EXIT_SUCCESS);
}

int record()
{
	arecord_init( SND_PCM_FORMAT_S16_LE, 1, 22050);
	s_fd = safe_open("/mnt/audio/test2.pcm");
	if (s_fd < 0) {
		perror("/mnt/audio/test2.pcm");
		prg_exit(EXIT_FAILURE);
	}
	u_char audiobuf[8192];
	while (!s_in_aborting) {
		size_t save = arecord_read(audiobuf,s_chunk_bytes);
		//size_t save = arecord_read(audiobuf,2048);
		if (save !=s_chunk_bytes){
			s_in_aborting = 1;
		}
			if (xwrite(s_fd, audiobuf, save) != save) {
				//perror(name);
				s_in_aborting = 1;
				break;
			}
	}
	//capture("/mnt/audio/test2.pcm", xwrite);
	arecord_uninit();
	return EXIT_SUCCESS;
}

static void show_available_sample_formats(snd_pcm_hw_params_t* params)
{
	snd_pcm_format_t format;

	fprintf(stderr, "Available formats:\n");
	for (format = 0; format <= SND_PCM_FORMAT_LAST; format++) {
		if (snd_pcm_hw_params_test_format(s_handle, params, format) == 0)
			fprintf(stderr, "- %s\n", snd_pcm_format_name(format));
	}
}

static int set_params(void)
{
	snd_pcm_hw_params_t *params;
	snd_pcm_sw_params_t *swparams;
	snd_pcm_uframes_t buffer_size;
	int err;
	size_t n;
	unsigned int rate;
	snd_pcm_uframes_t start_threshold, stop_threshold;
	snd_pcm_hw_params_alloca(&params);
	snd_pcm_sw_params_alloca(&swparams);
	err = snd_pcm_hw_params_any(s_handle, params);
	if (err < 0) {
		error(_("Broken configuration for this PCM: no configurations available"));
		return prg_exit(EXIT_FAILURE);
	}
	if (s_mmap_flag) {
		snd_pcm_access_mask_t *mask = alloca(snd_pcm_access_mask_sizeof());
		snd_pcm_access_mask_none(mask);
		snd_pcm_access_mask_set(mask, SND_PCM_ACCESS_MMAP_INTERLEAVED);
		snd_pcm_access_mask_set(mask, SND_PCM_ACCESS_MMAP_NONINTERLEAVED);
		snd_pcm_access_mask_set(mask, SND_PCM_ACCESS_MMAP_COMPLEX);
		err = snd_pcm_hw_params_set_access_mask(s_handle, params, mask);
	} else if (s_interleaved)
		err = snd_pcm_hw_params_set_access(s_handle, params,
						   SND_PCM_ACCESS_RW_INTERLEAVED);
	else
		err = snd_pcm_hw_params_set_access(s_handle, params,
						   SND_PCM_ACCESS_RW_NONINTERLEAVED);
	if (err < 0) {
		error(_("Access type not available"));
		return prg_exit(EXIT_FAILURE);
	}
	err = snd_pcm_hw_params_set_format(s_handle, params, s_hwparams.format);
	if (err < 0) {
		error(_("Sample format non available"));
		show_available_sample_formats(params);
		return prg_exit(EXIT_FAILURE);
	}
	err = snd_pcm_hw_params_set_channels(s_handle, params, s_hwparams.channels);
	if (err < 0) {
		error(_("Channels count non available"));
		return prg_exit(EXIT_FAILURE);
	}

	rate = s_hwparams.rate;
	err = snd_pcm_hw_params_set_rate_near(s_handle, params, &s_hwparams.rate, 0);
	assert(err >= 0);
	if ((float)rate * 1.05 < s_hwparams.rate || (float)rate * 0.95 > s_hwparams.rate) {
		if (!s_quiet_mode) {
			char plugex[64];
			const char *pcmname = snd_pcm_name(s_handle);
			fprintf(stderr, _("Warning: rate is not accurate (requested = %iHz, got = %iHz)\n"), rate, s_hwparams.rate);
			if (! pcmname || strchr(snd_pcm_name(s_handle), ':'))
				*plugex = 0;
			else
				snprintf(plugex, sizeof(plugex), "(-Dplug:%s)",
					 snd_pcm_name(s_handle));
			fprintf(stderr, _("         please, try the plug plugin %s\n"),
				plugex);
		}
	}
	rate = s_hwparams.rate;
	if (s_buffer_time == 0 && s_buffer_frames == 0) {
		err = snd_pcm_hw_params_get_buffer_time_max(params,
							    &s_buffer_time, 0);
		assert(err >= 0);
		if (s_buffer_time > 500000)
			s_buffer_time = 500000;
	}
	if (s_period_time == 0 && s_period_frames == 0) {
		if (s_buffer_time > 0)
			s_period_time = s_buffer_time / 4;
		else
			s_period_frames = s_buffer_frames / 4;
	}
	if (s_period_time > 0)
		err = snd_pcm_hw_params_set_period_time_near(s_handle, params,
							     &s_period_time, 0);
	else
		err = snd_pcm_hw_params_set_period_size_near(s_handle, params,
							     &s_period_frames, 0);
	assert(err >= 0);
	if (s_buffer_time > 0) {
		err = snd_pcm_hw_params_set_buffer_time_near(s_handle, params,
							     &s_buffer_time, 0);
	} else {
		err = snd_pcm_hw_params_set_buffer_size_near(s_handle, params,
							     &s_buffer_frames);
	}
	assert(err >= 0);
	s_monotonic = snd_pcm_hw_params_is_monotonic(params);
	s_can_pause = snd_pcm_hw_params_can_pause(params);
	err = snd_pcm_hw_params(s_handle, params);
	if (err < 0) {
		error(_("Unable to install hw params:"));
		snd_pcm_hw_params_dump(params, s_log);
		return prg_exit(EXIT_FAILURE);
	}
	snd_pcm_hw_params_get_period_size(params, &s_chunk_size, 0);
	snd_pcm_hw_params_get_buffer_size(params, &buffer_size);
	if (s_chunk_size == buffer_size) {
		error(_("Can't use period equal to buffer size (%lu == %lu)"),
		      s_chunk_size, buffer_size);
		return prg_exit(EXIT_FAILURE);
	}
	err = snd_pcm_sw_params_current(s_handle, swparams);
	if (err < 0) {
		error(_("Unable to get current sw params."));
		return prg_exit(EXIT_FAILURE);
	}
	if (s_avail_min < 0)
		n = s_chunk_size;
	else
		n = (double) rate * s_avail_min / 1000000;
	err = snd_pcm_sw_params_set_avail_min(s_handle, swparams, n);

	/* round up to closest transfer boundary */
	n = buffer_size;
	if (s_start_delay <= 0) {
		start_threshold = n + (double) rate * s_start_delay / 1000000;
	} else
		start_threshold = (double) rate * s_start_delay / 1000000;
	if (start_threshold < 1)
		start_threshold = 1;
	if (start_threshold > n)
		start_threshold = n;
	err = snd_pcm_sw_params_set_start_threshold(s_handle, swparams, start_threshold);
	assert(err >= 0);
	if (s_stop_delay <= 0)
		stop_threshold = buffer_size + (double) rate * s_stop_delay / 1000000;
	else
		stop_threshold = (double) rate * s_stop_delay / 1000000;
	err = snd_pcm_sw_params_set_stop_threshold(s_handle, swparams, stop_threshold);
	assert(err >= 0);

	if (snd_pcm_sw_params(s_handle, swparams) < 0) {
		error(_("unable to install sw params:"));
		snd_pcm_sw_params_dump(swparams, s_log);
		return prg_exit(EXIT_FAILURE);
	}

	if (s_verbose)
		snd_pcm_dump(s_handle, s_log);

	s_bits_per_sample = snd_pcm_format_physical_width(s_hwparams.format);
	s_significant_bits_per_sample = snd_pcm_format_width(s_hwparams.format);
	s_bits_per_frame = s_bits_per_sample * s_hwparams.channels;
	s_chunk_bytes = s_chunk_size * s_bits_per_frame / 8;

	// fprintf(stderr, "real s_chunk_size = %i, frags = %i, total = %i\n", s_chunk_size, setup.buf.block.frags, setup.buf.block.frags * s_chunk_size);

	/* stereo VU-meter isn't always available... */
	if (s_vumeter == VUMETER_STEREO) {
		if (s_hwparams.channels != 2 || !s_interleaved || s_verbose > 2)
			s_vumeter = VUMETER_MONO;
	}

	/* show mmap buffer arragment */
	if (s_mmap_flag && s_verbose) {
		const snd_pcm_channel_area_t *areas;
		snd_pcm_uframes_t offset, size = s_chunk_size;
		int i;
		err = snd_pcm_mmap_begin(s_handle, &areas, &offset, &size);
		if (err < 0) {
			error(_("snd_pcm_mmap_begin problem: %s"), snd_strerror(err));
			return prg_exit(EXIT_FAILURE);
		}
		for (i = 0; i < s_hwparams.channels; i++)
			fprintf(stderr, "mmap_area[%i] = %p,%u,%u (%u)\n", i, areas[i].addr, areas[i].first, areas[i].step, snd_pcm_format_physical_width(s_hwparams.format));
		/* not required, but for sure */
		snd_pcm_mmap_commit(s_handle, offset, 0);
	}

	s_buffer_frames = buffer_size;	/* for position test */
	return 0;
}

static void do_pause(void)
{
	int err;
	unsigned char b;

	if (!s_can_pause) {
		fprintf(stderr, _("\rPAUSE s_command ignored (no hw support)\n"));
		return;
	}
	if (snd_pcm_state(s_handle) == SND_PCM_STATE_SUSPENDED)
		suspend();

	err = snd_pcm_pause(s_handle, 1);
	if (err < 0) {
		error(_("pause push error: %s"), snd_strerror(err));
		return;
	}
	while (1) {
		while (read(fileno(stdin), &b, 1) != 1);
		if (b == ' ' || b == '\r') {
			while (read(fileno(stdin), &b, 1) == 1);
			if (snd_pcm_state(s_handle) == SND_PCM_STATE_SUSPENDED)
				suspend();
			err = snd_pcm_pause(s_handle, 0);
			if (err < 0)
				error(_("pause release error: %s"), snd_strerror(err));
			return;
		}
	}
}

static void check_stdin(void)
{
	unsigned char b;

	if (!s_interactive)
		return;
	if (s_fd != fileno(stdin)) {
		while (read(fileno(stdin), &b, 1) == 1) {
			if (b == ' ' || b == '\r') {
				while (read(fileno(stdin), &b, 1) == 1);
				fprintf(stderr, _("\r=== PAUSE ===                                                            "));
				fflush(stderr);
			do_pause();
				fprintf(stderr, "                                                                          \r");
				fflush(stderr);
			}
		}
	}
}

#ifndef timersub
#define	timersub(a, b, result) \
do { \
	(result)->tv_sec = (a)->tv_sec - (b)->tv_sec; \
	(result)->tv_usec = (a)->tv_usec - (b)->tv_usec; \
	if ((result)->tv_usec < 0) { \
		--(result)->tv_sec; \
		(result)->tv_usec += 1000000; \
	} \
} while (0)
#endif

#ifndef timermsub
#define	timermsub(a, b, result) \
do { \
	(result)->tv_sec = (a)->tv_sec - (b)->tv_sec; \
	(result)->tv_nsec = (a)->tv_nsec - (b)->tv_nsec; \
	if ((result)->tv_nsec < 0) { \
		--(result)->tv_sec; \
		(result)->tv_nsec += 1000000000L; \
	} \
} while (0)
#endif

/* I/O error s_handler */
static void xrun(void)
{
	snd_pcm_status_t *status;
	int res;

	snd_pcm_status_alloca(&status);
	if ((res = snd_pcm_status(s_handle, status))<0) {
		error(_("status error: %s"), snd_strerror(res));
		prg_exit(EXIT_FAILURE);
	}
	if (snd_pcm_status_get_state(status) == SND_PCM_STATE_XRUN) {
		if (s_fatal_errors) {
			error(_("fatal %s: %s"),
					stream == SND_PCM_STREAM_PLAYBACK ? _("underrun") : _("overrun"),
					snd_strerror(res));
			prg_exit(EXIT_FAILURE);
		}
		if (s_monotonic) {
#ifdef HAVE_CLOCK_GETTIME
			struct timespec now, diff, tstamp;
			clock_gettime(CLOCK_MONOTONIC, &now);
			snd_pcm_status_get_trigger_htstamp(status, &tstamp);
			timermsub(&now, &tstamp, &diff);
			fprintf(stderr, _("%s!!! (at least %.3f ms long)\n"),
				stream == SND_PCM_STREAM_PLAYBACK ? _("underrun") : _("overrun"),
				diff.tv_sec * 1000 + diff.tv_nsec / 1000000.0);
#else
			fprintf(stderr, "%s !!!\n", _("underrun"));
#endif
		} else {
			struct timeval now, diff, tstamp;
			gettimeofday(&now, 0);
			snd_pcm_status_get_trigger_tstamp(status, &tstamp);
			timersub(&now, &tstamp, &diff);
			fprintf(stderr, _("%s!!! (at least %.3f ms long)\n"),
				stream == SND_PCM_STREAM_PLAYBACK ? _("underrun") : _("overrun"),
				diff.tv_sec * 1000 + diff.tv_usec / 1000.0);
		}
		if (s_verbose) {
			fprintf(stderr, _("Status:\n"));
			snd_pcm_status_dump(status, s_log);
		}
		if ((res = snd_pcm_prepare(s_handle))<0) {
			error(_("xrun: prepare error: %s"), snd_strerror(res));
			prg_exit(EXIT_FAILURE);
		}
		return;		/* ok, data should be accepted again */
	} if (snd_pcm_status_get_state(status) == SND_PCM_STATE_DRAINING) {
		if (s_verbose) {
			fprintf(stderr, _("Status(DRAINING):\n"));
			snd_pcm_status_dump(status, s_log);
		}
		if (stream == SND_PCM_STREAM_CAPTURE) {
			fprintf(stderr, _("capture stream format change? attempting recover...\n"));
			if ((res = snd_pcm_prepare(s_handle))<0) {
				error(_("xrun(DRAINING): prepare error: %s"), snd_strerror(res));
				prg_exit(EXIT_FAILURE);
			}
			return;
		}
	}
	if (s_verbose) {
		fprintf(stderr, _("Status(R/W):\n"));
		snd_pcm_status_dump(status, s_log);
	}
	error(_("read/write error, state = %s"), snd_pcm_state_name(snd_pcm_status_get_state(status)));
	prg_exit(EXIT_FAILURE);
}

/* I/O suspend s_handler */
static void suspend(void)
{
	int res;

	if (!s_quiet_mode) {
		fprintf(stderr, _("Suspended. Trying resume. ")); fflush(stderr);
	}
	while ((res = snd_pcm_resume(s_handle)) == -EAGAIN)
		sleep(1);	/* wait until suspend flag is released */
	if (res < 0) {
		if (!s_quiet_mode) {
			fprintf(stderr, _("Failed. Restarting stream. ")); fflush(stderr);
		}
		if ((res = snd_pcm_prepare(s_handle)) < 0) {
			error(_("suspend: prepare error: %s"), snd_strerror(res));
			prg_exit(EXIT_FAILURE);
		}
	}
	if (!s_quiet_mode)
		fprintf(stderr, _("Done.\n"));
}

static void print_vu_meter_mono(int perc, int maxperc)
{
	const int bar_length = 50;
	char line[80];
	int val;

	for (val = 0; val <= perc * bar_length / 100 && val < bar_length; val++)
		line[val] = '#';
	for (; val <= maxperc * bar_length / 100 && val < bar_length; val++)
		line[val] = ' ';
	line[val] = '+';
	for (++val; val <= bar_length; val++)
		line[val] = ' ';
	if (maxperc > 99)
		sprintf(line + val, "| MAX");
	else
		sprintf(line + val, "| %02i%%", maxperc);
	fputs(line, stderr);
	if (perc > 100)
		fprintf(stderr, _(" !clip  "));
}

static void print_vu_meter_stereo(int *perc, int *maxperc)
{
	const int bar_length = 35;
	char line[80];
	int c;

	memset(line, ' ', sizeof(line) - 1);
	line[bar_length + 3] = '|';

	for (c = 0; c < 2; c++) {
		int p = perc[c] * bar_length / 100;
		char tmp[4];
		if (p > bar_length)
			p = bar_length;
		if (c)
			memset(line + bar_length + 6 + 1, '#', p);
		else
			memset(line + bar_length - p - 1, '#', p);
		p = maxperc[c] * bar_length / 100;
		if (p > bar_length)
			p = bar_length;
		if (c)
			line[bar_length + 6 + 1 + p] = '+';
		else
			line[bar_length - p - 1] = '+';
		if (ABS(maxperc[c]) > 99)
			sprintf(tmp, "MAX");
		else
			sprintf(tmp, "%02d%%", maxperc[c]);
		if (c)
			memcpy(line + bar_length + 3 + 1, tmp, 3);
		else
			memcpy(line + bar_length, tmp, 3);
	}
	line[bar_length * 2 + 6 + 2] = 0;
	fputs(line, stderr);
}

static void print_vu_meter(signed int *perc, signed int *maxperc)
{
	if (s_vumeter == VUMETER_STEREO)
		print_vu_meter_stereo(perc, maxperc);
	else
		print_vu_meter_mono(*perc, *maxperc);
}

/* peak s_handler */
static void compute_max_peak(u_char *data, size_t samples)
{
	signed int val, max, perc[2], max_peak[2];
	static int run = 0;
	size_t osamples = samples;
	int format_little_endian = snd_pcm_format_little_endian(s_hwparams.format);
	int ichans, c;

	if (s_vumeter == VUMETER_STEREO)
		ichans = 2;
	else
		ichans = 1;

	memset(max_peak, 0, sizeof(max_peak));
	switch (s_bits_per_sample) {
	case 8: {
		signed char *valp = (signed char *)data;
		signed char mask = snd_pcm_format_silence(s_hwparams.format);
		c = 0;
		while (samples-- > 0) {
			val = *valp++ ^ mask;
			val = abs(val);
			if (max_peak[c] < val)
				max_peak[c] = val;
			if (s_vumeter == VUMETER_STEREO)
				c = !c;
		}
		break;
	}
	case 16: {
		signed short *valp = (signed short *)data;
		signed short mask = snd_pcm_format_silence_16(s_hwparams.format);
		signed short sval;

		c = 0;
		while (samples-- > 0) {
			if (format_little_endian)
				sval = le16toh(*valp);
			else
				sval = be16toh(*valp);
			sval = abs(sval) ^ mask;
			if (max_peak[c] < sval)
				max_peak[c] = sval;
			valp++;
			if (s_vumeter == VUMETER_STEREO)
				c = !c;
		}
		break;
	}
	case 24: {
		unsigned char *valp = data;
		signed int mask = snd_pcm_format_silence_32(s_hwparams.format);

		c = 0;
		while (samples-- > 0) {
			if (format_little_endian) {
				val = valp[0] | (valp[1]<<8) | (valp[2]<<16);
			} else {
				val = (valp[0]<<16) | (valp[1]<<8) | valp[2];
			}
			/* Correct signed bit in 32-bit value */
			if (val & (1<<(s_bits_per_sample-1))) {
				val |= 0xff<<24;	/* Negate upper bits too */
			}
			val = abs(val) ^ mask;
			if (max_peak[c] < val)
				max_peak[c] = val;
			valp += 3;
			if (s_vumeter == VUMETER_STEREO)
				c = !c;
		}
		break;
	}
	case 32: {
		signed int *valp = (signed int *)data;
		signed int mask = snd_pcm_format_silence_32(s_hwparams.format);

		c = 0;
		while (samples-- > 0) {
			if (format_little_endian)
				val = le32toh(*valp);
			else
				val = be32toh(*valp);
			val = abs(val) ^ mask;
			if (max_peak[c] < val)
				max_peak[c] = val;
			valp++;
			if (s_vumeter == VUMETER_STEREO)
				c = !c;
		}
		break;
	}
	default:
		if (run == 0) {
			fprintf(stderr, _("Unsupported bit size %d.\n"), (int)s_bits_per_sample);
			run = 1;
		}
		return;
	}
	max = 1 << (s_significant_bits_per_sample-1);
	if (max <= 0)
		max = 0x7fffffff;

	for (c = 0; c < ichans; c++) {
		if (s_bits_per_sample > 16)
			perc[c] = max_peak[c] / (max / 100);
		else
			perc[c] = max_peak[c] * 100 / max;
	}

	if (s_interleaved && s_verbose <= 2) {
		static int maxperc[2];
		static time_t t=0;
		const time_t tt=time(NULL);
		if(tt>t) {
			t=tt;
			maxperc[0] = 0;
			maxperc[1] = 0;
		}
		for (c = 0; c < ichans; c++)
			if (perc[c] > maxperc[c])
				maxperc[c] = perc[c];

		putc('\r', stderr);
		print_vu_meter(perc, maxperc);
		fflush(stderr);
	}
	else if (s_verbose==3) {
		fprintf(stderr, _("Max peak (%li samples): 0x%08x "), (long)osamples, max_peak[0]);
		for (val = 0; val < 20; val++)
			if (val <= perc[0] / 5)
				putc('#', stderr);
			else
				putc(' ', stderr);
		fprintf(stderr, " %i%%\n", perc[0]);
		fflush(stderr);
	}
}

/*
 *  read function
 */

static ssize_t pcm_read(u_char *data, size_t rcount)
{
	ssize_t r;
	size_t result = 0;
	size_t count = rcount;

	if (count != s_chunk_size) {
		count = s_chunk_size;
	}

	while (count > 0) {
		if (s_in_aborting)
			goto abort;
		//check_stdin();
		r = readi_func(s_handle, data, count);
		if (r == -EAGAIN || (r >= 0 && (size_t)r < count)) {
			if (!s_test_nowait)
				snd_pcm_wait(s_handle, 100);
		} else if (r == -EPIPE) {
			xrun();
		} else if (r == -ESTRPIPE) {
			suspend();
		} else if (r < 0) {
			error(_("read error: %s"), snd_strerror(r));
			prg_exit(EXIT_FAILURE);
		}
		if (r > 0) {
			if (s_vumeter)
				compute_max_peak(data, r * s_hwparams.channels);
			result += r;
			count -= r;
			data += r * s_bits_per_frame / 8;
		}
	}
abort:
	return rcount;
}

/* that was a big one, perhaps somebody split it :-) */

/* setting the globals for playing raw data */
static void init_raw_data(void)
{
	s_hwparams = s_rs_hwparams;
}

/* calculate the data count to read from/to dsp */
static off64_t calc_count(void)
{
	off64_t count;

	if (s_timelimit == 0)
		if (s_sampleslimit == 0)
			count = s_pbrec_count;
		else
			count = snd_pcm_format_size(s_hwparams.format, s_sampleslimit * s_hwparams.channels);
	else {
		count = snd_pcm_format_size(s_hwparams.format, s_hwparams.rate * s_hwparams.channels);
		count *= (off64_t)s_timelimit;
	}
	return count < s_pbrec_count ? count : s_pbrec_count;
}

/**
 * create_path
 *
 *   This function creates a file path, like mkdir -p.
 *
 * Parameters:
 *
 *   path - the path to create
 *
 * Returns: 0 on success, -1 on failure
 * On failure, a message has been printed to stderr.
 */
int create_path(const char *path)
{
	char *start;
	mode_t mode = S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH;

	if (path[0] == '/')
		start = strchr(path + 1, '/');
	else
		start = strchr(path, '/');

	while (start) {
		char *buffer = strdup(path);
		buffer[start-path] = 0x00;

		if (mkdir(buffer, mode) == -1 && errno != EEXIST) {
			fprintf(stderr, "Problem creating directory %s", buffer);
			perror(" ");
			free(buffer);
			return -1;
		}
		free(buffer);
		start = strchr(start + 1, '/');
	}
	return 0;
}

static int safe_open(const char *name)
{
	int fd;

	fd = open(name, O_WRONLY | O_CREAT, 0644);
	if (fd == -1) {
		if (errno != ENOENT || !s_use_strftime)
			return -1;
		if (create_path(name) == 0)
			fd = open(name, O_WRONLY | O_CREAT, 0644);
	}
	return fd;
}

int arecord_read(u_char *buf,  int len)
{
	size_t f = len * 8 / s_bits_per_frame;
	size_t read = pcm_read(buf, f);
	size_t save = read * s_bits_per_frame / 8;
	return save;
}

#endif