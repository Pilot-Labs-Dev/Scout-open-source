#pragma once
#include <alsa/asoundlib.h>

#ifdef __cplusplus
extern "C"{
int arecord_read(u_char *buf,  int len);
int arecord_init(snd_pcm_format_t format,unsigned int channels,unsigned int rate);
void arecord_uninit();
}
#else
int arecord_read(u_char *buf,  int len);
int arecord_init(snd_pcm_format_t format,unsigned int channels,unsigned int rate);
void arecord_uninit();
#endif