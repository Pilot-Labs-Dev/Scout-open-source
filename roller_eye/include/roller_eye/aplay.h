#pragma once
#include <alsa/asoundlib.h>
#ifdef __cplusplus
extern "C"{
int  aplay_init(snd_pcm_format_t format, unsigned int channels,	unsigned int rate);
void aplay_unInit();
int  aplay_play(u_char *data ,int len);
}
#else
int  aplay_init(snd_pcm_format_t format, unsigned int channels,	unsigned int rate);
void aplay_unInit();
int  aplay_play(u_char *data ,int len);
#endif