int8 VIDEO_STREAM_H264=0
int8 VIDEO_STREAM_JPG=1
int8 AUDIO_STREAM_AAC=2

#common frame for video/audio stream,compress picture
#do not use common head,define a new header
uint32 seq
uint64 stamp
uint32 session
int8 type

# origin frame seq
uint32 oseq           

#in video frame,par1 is width,par2 is height,par3 is key frame
#in audio  frame,par1 is sample rate,par2 is bitwidth,par3 is channels
#in file frame,par3 is frame flag:0->frame  1->eof  2->error
#par4 used to extend
int32 par1
int32 par2
int32 par3
int32 par4

#frame data
uint8[] data
