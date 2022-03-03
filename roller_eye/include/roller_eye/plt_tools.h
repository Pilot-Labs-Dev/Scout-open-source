#ifndef __PLT_TOOLS___H__
#define __PLT_TOOLS___H__
#include<stdint.h>
#include <termios.h>
#include<sys/time.h>
#ifdef __cplusplus
extern "C"{
#endif

#pragma pack(1)
 
/*14Bytes*/
typedef struct                       /**** BMP file header structure ****/  
{
    unsigned short bfType;
	unsigned int   bfSize;           /* Size of file */  
	unsigned short bfReserved1;      /* Reserved */  
	unsigned short bfReserved2;      /* ... */  
	unsigned int   bfOffBits;        /* Offset to bitmap data */  
} BMP_FILE_HEADER;
 
/*40Bytes*/
typedef struct                       /**** BMP file info structure ****/  
{
	unsigned int   biSize;           /* Size of info header */
	int            biWidth;          /* Width of image */  
	int            biHeight;         /* Height of image */  
	unsigned short biPlanes;         /* Number of color planes */  
	unsigned short biBitCount;       /* Number of bits per pixel */  
	unsigned int   biCompression;    /* Type of compression to use */  
	unsigned int   biSizeImage;      /* Size of image data */  
	int            biXPelsPerMeter;  /* X pixels per meter */  
	int            biYPelsPerMeter;  /* Y pixels per meter */  
	unsigned int   biClrUsed;        /* Number of colors used */  
	unsigned int   biClrImportant;   /* Number of important colors */  
} BMP_INFO_HEADER;
 
#pragma pack()



int rgbToBmpFile(const char *pFileName, unsigned char* pRgbaData, int nWidth , int nHeight);
int clipRgbaToBmpFile(const char *pFileName, unsigned char* pRgbaData, int nWidth , int nHeight,
    int nMarkLeft, int nMarkTop, int nMarkWidth, int nMarkHeight);
int markPosToBmpFile(char *pFileName, unsigned char* pRgbaData, int nWidth , int nHeight,
    int nMarkTop, int nMarkLeft, int nMarkBottom, int nMarkRight);
uint64_t  plt_get_mono_time_ms();
int  plt_timeval_diff_ms(struct timeval *t1,struct timeval *t2);
int64_t  plt_timeval_diff_us(struct timeval *t1,struct timeval *t2);
int64_t  plt_timeval_to_ns(struct timeval *t);
int getFileNumByType(const char* path, const char* fileType);
int  set_serial_attrib (
    int fd,
    int  baudrate,          // B1200 B2400 B4800 B9600 .. B115200
    int  databit,           // 5, 6, 7, 8
    const char *stopbit,    //  "1", "1.5", "2"
    char parity,            // N(o), O(dd), E(ven)
    int vtime,
    int vmin );

int change_file_mode(const char* path,const char* mode);

int mk_depth_dir(const char* path);

void I422toI420(const unsigned char *YUV422, unsigned char *y,unsigned char *u, unsigned char *v, int width,  int height);

void yuyv2I420(const unsigned char *yuyv, unsigned char *y,unsigned char *u, unsigned char *v,  int width,  int height);

void copyYFromYUYV(const unsigned char *yuyv, unsigned char *y, int width,  int height);

int checkTimeSynced(int waitSync);

int open_lock_whole_file(const char* path);

int close_unlock_whole_file(int fd);

int download_file(const char* uri,const char* path);

int plt_system(const char *cmd);

void mid_wifi_ssid_convert_utf8(unsigned char *ssid, const char *bssid, int size);

int get_gateway(char *gateway);

int ping_status(char *ip);

#ifdef __cplusplus
}
#endif
#endif