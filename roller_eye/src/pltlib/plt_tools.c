#include<time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include<sys/fcntl.h>
#include <dirent.h>
#include<time.h>
#include<unistd.h>
#include"plt_tools.h"
#include "plterrno.h"
#include"system_define.h"

#include <arpa/inet.h>
#include <linux/rtnetlink.h>
#include <net/if.h>
#include <unistd.h>

typedef char bool;

#define BUFSIZE 8192


uint64_t  plt_get_mono_time_ms()
{
    struct timespec s;

    if(clock_gettime(CLOCK_MONOTONIC,&s)<0)
    {
        return 0;
    }
    return (uint64_t)s.tv_sec*1000+(uint64_t)s.tv_nsec/1000000;
}

int  plt_timeval_diff_ms(struct timeval *t1,struct timeval *t2)
{
    return (int)((t1->tv_sec-t2->tv_sec)*1000+(t1->tv_usec-t2->tv_usec)/1000);
}

int64_t  plt_timeval_diff_us(struct timeval *t1,struct timeval *t2)
{
    return (int64_t)(t1->tv_sec-t2->tv_sec)*1000000+(int64_t)(t1->tv_usec-t2->tv_usec);
}
int64_t  plt_timeval_to_ns(struct timeval *t)
{
    return (int64_t)(t->tv_sec)*1000000000+(int64_t)(t->tv_usec)*1000;
}

int rgbToBmpFile(const char *pFileName, unsigned char* pRgbaData, int nWidth , int nHeight)
{
    #define BITS_PER_PIXEL 24

    BMP_FILE_HEADER bmpHeader;
    BMP_INFO_HEADER bmpInfo;

    FILE* fp         = NULL;
    char* pBmpSource = NULL;
    char* pBmpData   = NULL;

    int i = 0, j=0;

    int bytesPerLine = (nWidth*BITS_PER_PIXEL+31)/32*4;
    //int bytesPerLine = (nWidth*BITS_PER_PIXEL+31)/8;
    int pixelBytes  = bytesPerLine*nHeight;

    bmpHeader.bfType        = 0x4D42; //BM
    bmpHeader.bfReserved1   = 0;
    bmpHeader.bfReserved2   = 0;
    bmpHeader.bfOffBits     = sizeof(BMP_FILE_HEADER) + sizeof(BMP_INFO_HEADER);
    bmpHeader.bfSize        = bmpHeader.bfOffBits     + pixelBytes;

    bmpInfo.biSize          = sizeof(BMP_INFO_HEADER);
    bmpInfo.biWidth         = nWidth;
    //left-right up-down
    bmpInfo.biHeight        = -nHeight;
    bmpInfo.biPlanes        = 1;
    bmpInfo.biBitCount      = BITS_PER_PIXEL;
    bmpInfo.biCompression   = 0;
    bmpInfo.biSizeImage     = pixelBytes;
    bmpInfo.biXPelsPerMeter = 100;
    bmpInfo.biYPelsPerMeter = 100;
    bmpInfo.biClrUsed       = 0;
    bmpInfo.biClrImportant  = 0;

    pBmpSource = (char *)malloc(pixelBytes);
    if (!pBmpSource) {
        perror("malloc");
        return -1;
    }

    fp = fopen(pFileName,"wb+");
    if (!fp) {
        perror("fopen");
        return -1;
    }

    fwrite(&bmpHeader, sizeof(BMP_FILE_HEADER), 1, fp);
    fwrite(&bmpInfo,   sizeof(BMP_INFO_HEADER), 1, fp);

    pBmpData = pBmpSource;
    for (i=0; i<nHeight; i++) {
        for (j=0; j<nWidth; j++) {
           pBmpData[0] = pRgbaData[2]; //BGR->RGB
           pBmpData[1] = pRgbaData[1];
           pBmpData[2] = pRgbaData[0];
	       pRgbaData += 3;
           pBmpData  += 3;
        }
        pBmpData += (bytesPerLine - nWidth*3);
    }
    fwrite(pBmpSource, pixelBytes, 1, fp);

    fclose(fp);
    free(pBmpSource);

    return 0;
}

int clipRgbaToBmpFile(const char *pFileName, unsigned char* pRgbaData, int nWidth , int nHeight,
    int nMarkLeft, int nMarkTop, int nMarkWidth, int nMarkHeight)
{
    unsigned char* pClipSource     = NULL;
    unsigned char* pClipData       = NULL;
    int pixelBytes       = nMarkWidth*nMarkHeight*4;
    int i = 0;

    pClipSource = (unsigned char *)malloc(pixelBytes);
    if (!pClipSource) {
        return -1;
    }

    pRgbaData += nMarkTop * nWidth * 4;
    pRgbaData += nMarkLeft * 4;

    pClipData = pClipSource;
    for (i=0; i<nMarkHeight; i++) {
        memcpy(pClipData, pRgbaData, nMarkWidth*4);
        pRgbaData += nWidth    * 4;
        pClipData += nMarkWidth* 4;
    }

    rgbToBmpFile(pFileName, pClipSource, nMarkWidth, nMarkHeight);

    free(pClipSource);

    return 0;
}

static unsigned char * paintRect(unsigned char *pData, int nWidth, int nHeight, int nPicWidth)
{
    int i, j;
    for (i = 0; i < nHeight; i++) {
        for (j = 0; j < nWidth; j++) {
            pData[0] = 0xFF;
            pData[1] = 0;
            pData[2] = 0;
	        pData += 3;
        }
        pData += (nPicWidth - nWidth) * 3;
    }

    return pData;
}

int markPosToBmpFile(char *pFileName, unsigned char* pRgbaData, int nWidth , int nHeight,
    int nMarkTop, int nMarkLeft, int nMarkBottom, int nMarkRight)
{
    const int LINE_SIZE = 2;
    unsigned char* pClipSource = pRgbaData;
    unsigned char* pZone1Data;
    unsigned char* pZone2Data;
    unsigned char* pZone3Data;
    unsigned char* pZone4Data;

    if (nMarkTop < 0) {
        nMarkTop = 0;
    } else if (nMarkTop > nHeight) {
        printf("Illegal nMarkTop: %d\n", nMarkTop);
        return -1;
    }

    if (nMarkLeft < 0) {
        nMarkLeft = 0;
    } else if (nMarkLeft > nWidth) {
        printf("Illegal nMarkLeft: %d\n", nMarkLeft);
        return -1;
    }

    if (nMarkBottom < 0) {
        printf("Illegal nMarkBottom: %d\n", nMarkBottom);
        return -1;
    } else if (nMarkBottom > nHeight) {
        nMarkBottom = nHeight;
    }

    if (nMarkRight < 0) {
        printf("Illegal nMarkRight: %d\n", nMarkRight);
        return -1;
    } else if (nMarkRight > nWidth) {
        nMarkRight = nWidth;
    }
/*
    1
  -----
 2|   |3
  -----
    4
*/
    int nMarkWidth = nMarkRight - nMarkLeft;
    int nMarkHeight = nMarkBottom - nMarkTop;

    //Move to zone 1
    pRgbaData += nMarkTop * nWidth * 3;
    pRgbaData += nMarkLeft * 3;

    pZone1Data = pRgbaData;
    //Paint zone 1
    pRgbaData = paintRect(pZone1Data, nMarkWidth, LINE_SIZE, nWidth);

    pZone2Data = pRgbaData;
    pZone3Data = pRgbaData + (nMarkWidth - LINE_SIZE) * 3;

    //Paint zone 2
    pRgbaData = paintRect(pZone2Data, LINE_SIZE, nMarkHeight - LINE_SIZE * 2, nWidth);

    //Paint zone 3
    paintRect(pZone3Data, LINE_SIZE, nMarkHeight - LINE_SIZE * 2, nWidth);
    pZone4Data = pRgbaData;

    //Paint zone 4
    paintRect(pZone4Data, nMarkWidth, LINE_SIZE, nWidth);

    return rgbToBmpFile(pFileName, pClipSource, nWidth, nHeight);
}

static unsigned char getFileType(const char* type)
{
    if(strcmp(type,"b")==0)
    {
        return DT_BLK;
    }else if(strcmp(type,"c")==0)
    {
        return DT_CHR;
    }else if(strcmp(type,"d")==0)
    {
        return DT_DIR;
    }else if(strcmp(type,"p")==0)
    {
        return DT_FIFO;
    }else if(strcmp(type,"l")==0)
    {
        return DT_LNK;
    }else if(strcmp(type,"-")==0)
    {
        return DT_REG;
    }else if(strcmp(type,"s")==0)
    {
        return DT_SOCK;
    }else
    {
        printf("Invalid file type: %s\n", type);
        return DT_UNKNOWN;
    }

}

//fileType [-,d,p,s,l,c,b]
int getFileNumByType(const char* path, const char* fileType)
{
    int total = 0;
    DIR* dir = opendir(path);
    if(dir==NULL)
    {
        perror("opendir error");
        exit(1);
    }
    struct dirent* dirObj = NULL;
    char* dirName = NULL;

    while((dirObj = readdir(dir)) != NULL)
    {
        dirName = dirObj->d_name;
        if(strcmp(dirName,".")==0 || strcmp(dirName,"..")==0)
        {
            continue;
        }
        if(dirObj->d_type == getFileType(fileType))
        {
            total++;
        }
        if(dirObj->d_type==DT_DIR)
        {
            char temp[1024] = {0};
            sprintf(temp,"%s/%s",path,dirName);
           total+= getFileNumByType(temp, fileType);
        }
    }

    return total;
}
static int set_baudrate (struct termios *opt, unsigned int baudrate)
{
	if(cfsetispeed(opt, baudrate)!=0){
        return -1;
    }
	return cfsetospeed(opt, baudrate);
}
static void set_data_bit (struct termios *opt, unsigned int databit)
{
    opt->c_cflag &= ~CSIZE;
    switch (databit) {
    case 8:
        opt->c_cflag |= CS8;
        break;
    case 7:
        opt->c_cflag |= CS7;
        break;
    case 6:
        opt->c_cflag |= CS6;
        break;
    case 5:
        opt->c_cflag |= CS5;
        break;
    default:
        opt->c_cflag |= CS8;
        break;
    }
}
static void set_parity (struct termios *opt, char parity)
{
    switch (parity) {
    case 'N':                  /* no parity check */
        opt->c_cflag &= ~PARENB;
        break;
    case 'E':                  /* even */
        opt->c_cflag |= PARENB;
        opt->c_cflag &= ~PARODD;
        break;
    case 'O':                  /* odd */
        opt->c_cflag |= PARENB;
        opt->c_cflag |= ~PARODD;
        break;
    default:                   /* no parity check */
        opt->c_cflag &= ~PARENB;
        break;
    }
}
static void set_stopbit (struct termios *opt, const char *stopbit)
{
    if (0 == strcmp (stopbit, "1")) {
        opt->c_cflag &= ~CSTOPB; /* 1 stop bit */
    }	else if (0 == strcmp (stopbit, "1")) {
        opt->c_cflag &= ~CSTOPB; /* 1.5 stop bit */
    }   else if (0 == strcmp (stopbit, "2")) {
        opt->c_cflag |= CSTOPB;  /* 2 stop bits */
    } else {
        opt->c_cflag &= ~CSTOPB; /* 1 stop bit */
    }
}
int  set_serial_attrib (int fd, int  baudrate,  int  databit,  const char *stopbit, char parity, int vtime, int vmin )
{
     struct termios opt;
     if(tcgetattr(fd, &opt)!=0){
         return -1;
     }
     if(set_baudrate(&opt, baudrate)!=0){
         return -1;
     }

     opt.c_cflag 		 |= CLOCAL | CREAD;      /* | CRTSCTS */
     set_data_bit(&opt, databit);
     set_parity(&opt, parity);
     set_stopbit(&opt, stopbit);

     opt.c_oflag 		 = 0;
     opt.c_lflag            	|= 0;
     opt.c_oflag          	&= ~OPOST;
     opt.c_cc[VTIME]     	 = vtime;
     opt.c_cc[VMIN]         	 = vmin;
     tcflush (fd, TCIFLUSH);
     return (tcsetattr (fd, TCSANOW, &opt));
}
int change_file_mode(const char* path,const char* mode)
{
    char buf[128];
    int n;
    n=snprintf(buf,sizeof(buf),CMD_PREFIX"chmod %s %s",mode,path);
    if(n>=sizeof(buf)-1){//over flow
        pset_errno(PEFULL);
        return -1;
    }
    return system(buf);
}
int mk_depth_dir(const char* path)
{
    char buf[128];
    int n;
    n=snprintf(buf,sizeof(buf),"mkdir -p %s",path);
    if(n>=sizeof(buf)-1){//over flow
        pset_errno(PEFULL);
        return -1;
    }
    return system(buf);
}
void I422toI420(const unsigned char *YUV422, unsigned char *y,unsigned char *u, unsigned char *v, int width,  int height)
{
	const unsigned char *Y, *U, *V;
	unsigned char * Y2, *U2, *V2;
	int halfHeight = height>>1;

	Y = YUV422;
	U = Y + width*height;
	V = U + (width*height>>1);

	Y2 = y;
	U2 = u;
	V2 = v;

	int i = 0;
	for(i = 0; i < halfHeight; ++i)
	{
		memcpy(U2, U, width>>1);
		memcpy(V2, V, width>>1);

		U2 = U2 + (width>>1);
		V2 = V2 + (width>>1);

		U = U + (width);
		V = V + (width);
	}

	memcpy(Y2, Y, width*height);
}
void yuyv2I420(const unsigned char *yuyv, unsigned char *y,unsigned char *u, unsigned char *v,  int width,  int height)
{
    const unsigned char *Y;
	unsigned char *U;
	unsigned char *V;
    int size=width*height*2,i,j;
    int halfHeight = height/2;
    int doublewidth=width*2;

    Y = yuyv;
	for(i=0;i<size;i+=2)
	{
		*y++=*Y;
        Y+=2;
	}

    yuyv++;
    U = u;
	V = v;
    for(i=0;i<halfHeight;i++)
    {
        for(j=0;j<doublewidth;j+=4)
        {
            *U++=*yuyv;
            *V++=yuyv[2];
            yuyv+=4;
        }
        yuyv+=doublewidth;
    }
}
void copyYFromYUYV(const unsigned char *yuyv, unsigned char *y, int width,  int height)
{
    const unsigned char *Y;
    int size=width*height*2,i;

    Y = yuyv;
	for(i=0;i<size;i+=2)
	{
		*y++=*Y;
        Y+=2;
	}
}
int checkTimeSynced(int waitSync)
{
    static int synced=0;

    while (!synced)
    {
        synced=1;
        if(synced || !waitSync){
            break;
        }
        sleep(1);
    }

    return synced;
}
int open_lock_whole_file(const char* path)
{
    int fd=open(path,O_CREAT|O_RDWR,0600);
    if(fd<0){
        return fd;
    }
    if(lockf(fd,F_LOCK,0)<0){
        close(fd);
        return -1;
    }
    return fd;
}
int close_unlock_whole_file(int fd)
{
    int ret;
    if(fd<0){
        return -1;
    }
    ret=lockf(fd,F_ULOCK,0);
    close(fd);
    return ret;
}
int plt_system(const char *cmd)
{
    int ret=system(cmd);
    if(ret<0){
        return -1;
    }

    if(WIFEXITED(ret) && WEXITSTATUS(ret)==0){
        return 0;
    }else{
        return -1;
    }
}
int download_file(const char* uri,const char* path)
{
    char cmd[256];
    if(snprintf(cmd,sizeof(cmd),CMD_PREFIX"wget -O %s %s",path,uri)>=((int)sizeof(cmd)-1)){
        pset_errno(PEFULL);
        return -1;
    }
    return plt_system(cmd);
}

unsigned char mid_wifi_ssid_convert_decimal(char ssid)
{
    unsigned char ssid_char = 0;
    if (ssid >= '0' && ssid <='9'){
        ssid_char = ssid - '0';
    }else if (ssid >= 'a' && ssid <='f'){
        ssid_char = ssid - 'a' + 10;
    }else if (ssid >= 'A' && ssid <='F'){
        ssid_char = ssid - 'A' + 10;
    }else{
        ssid_char = 0;
    }
    return ssid_char;
}

void mid_wifi_ssid_convert_utf8(unsigned char *ssid, const char *bssid, int size)
{
    int ssid_char;
    int i=0, j=0;
    bool bcheck;

    for (i=0; i < size; i++){
        if (bssid[i] == '\0' || j >= 256){
            printf("%s  end j = %d \n", __func__, j);
            break;
        }
        bcheck = ((bssid[i] == '\\' && bssid[i+1] == 'x') || (bssid[i] == '\\' && bssid[i+1] == 'X'));
        if ( bcheck
            && (bssid[i+2] >= '0' && bssid[i+2] <='f')
            && (bssid[i+3] >= '0' && bssid[i+3] <='f')){
            ssid_char = (mid_wifi_ssid_convert_decimal(bssid[i+2]) << 4) + mid_wifi_ssid_convert_decimal(bssid[i+3]);
            if (ssid_char <= 255){
                ssid[j] = ssid_char;
            }else{
                ssid[j] = 0;
                printf("%s invalid ssid info. ssid_char = %d\n", __func__, ssid_char);
            }
            i += 3;
            j++;
        }else{
            ssid[j] = bssid[i];
            j++;
        }
    }
    ssid[j] = '\0';
}

struct route_info{
u_int dstAddr;
u_int srcAddr;
u_int gateWay;
char ifName[IF_NAMESIZE];
};


int readNlSock(int sockFd, char *bufPtr, int seqNum, int pId)
{
    struct nlmsghdr *nlHdr;
    int readLen = 0, msgLen = 0;

    do{
        if((readLen = recv(sockFd, bufPtr, BUFSIZE - msgLen, 0)) < 0){
            perror("SOCK READ: ");
            return -1;
        }

        nlHdr = (struct nlmsghdr *)bufPtr;

        if((NLMSG_OK(nlHdr, readLen) == 0) ||
                (nlHdr->nlmsg_type == NLMSG_ERROR)){
            perror("Error in recieved packet");
            return -1;
        }

        if(nlHdr->nlmsg_type == NLMSG_DONE){
            break;
        }else{
            bufPtr += readLen;
            msgLen += readLen;
        }

        if((nlHdr->nlmsg_flags & NLM_F_MULTI) == 0){
            break;
        }
    }while((nlHdr->nlmsg_seq != seqNum) || (nlHdr->nlmsg_pid != pId));

    return msgLen;
}

void parseRoutes(struct nlmsghdr *nlHdr, struct route_info *rtInfo,char *gateway)
{
    struct rtmsg *rtMsg;
    struct rtattr *rtAttr;
    int rtLen;
    char *tempBuf = NULL;
    struct in_addr dst;
    struct in_addr gate;
    tempBuf = (char *)malloc(100);
    rtMsg = (struct rtmsg *)NLMSG_DATA(nlHdr);

    // If the route is not for AF_INET or does not belong to main routing table
    //then return.
    if((rtMsg->rtm_family != AF_INET) ||
            (rtMsg->rtm_table != RT_TABLE_MAIN)){
        return;
    }

    rtAttr = (struct rtattr *)RTM_RTA(rtMsg);
    rtLen = RTM_PAYLOAD(nlHdr);

    for(;RTA_OK(rtAttr,rtLen); rtAttr = RTA_NEXT(rtAttr,rtLen)){
        switch(rtAttr->rta_type){
            case RTA_OIF:
                if_indextoname(*(int *)RTA_DATA(rtAttr), rtInfo->ifName);
            break;
            case RTA_GATEWAY:
                rtInfo->gateWay = *(u_int *)RTA_DATA(rtAttr);
            break;
            case RTA_PREFSRC:
                rtInfo->srcAddr = *(u_int *)RTA_DATA(rtAttr);
            break;
            case RTA_DST:
                rtInfo->dstAddr = *(u_int *)RTA_DATA(rtAttr);
            break;
        }
    }

    dst.s_addr = rtInfo->dstAddr;
    if(strstr((char *)inet_ntoa(dst), "0.0.0.0")){
        printf("oif:%s",rtInfo->ifName);
        gate.s_addr = rtInfo->gateWay;
        sprintf(gateway, (char *)inet_ntoa(gate));
        printf("gw%s\n",gateway);
        gate.s_addr = rtInfo->srcAddr;
        printf("src:%s\n",(char *)inet_ntoa(gate));
        gate.s_addr = rtInfo->dstAddr;
        printf("dst:%s\n",(char *)inet_ntoa(gate));
    }
    free(tempBuf);
    return;
}


int get_gateway(char *gateway)
{
    struct nlmsghdr *nlMsg;
    struct rtmsg *rtMsg;
    struct route_info *rtInfo;
    char msgBuf[BUFSIZE];
    int sock, len, msgSeq = 0;

    if((sock = socket(PF_NETLINK, SOCK_DGRAM, NETLINK_ROUTE)) < 0){
        perror("Socket Creation: ");
        return -1;
    }

    memset(msgBuf, 0, BUFSIZE);
    nlMsg = (struct nlmsghdr *)msgBuf;
    rtMsg = (struct rtmsg *)NLMSG_DATA(nlMsg);
    nlMsg->nlmsg_len = NLMSG_LENGTH(sizeof(struct rtmsg)); // Length of message.
    nlMsg->nlmsg_type = RTM_GETROUTE; // Get the routes from kernel routing table .
    nlMsg->nlmsg_flags = NLM_F_DUMP | NLM_F_REQUEST; // The message is a request for dump.
    nlMsg->nlmsg_seq = msgSeq++; // Sequence of the message packet.
    nlMsg->nlmsg_pid = getpid(); // PID of process sending the request.

    if(send(sock, nlMsg, nlMsg->nlmsg_len, 0) < 0){
        printf("Write To Socket Failed…\n");
        return -1;
    }


    if((len = readNlSock(sock, msgBuf, msgSeq, getpid())) < 0){
        printf("Read From Socket Failed…\n");
        return -1;
    }

    rtInfo = (struct route_info *)malloc(sizeof(struct route_info));

    for(;NLMSG_OK(nlMsg,len);nlMsg = NLMSG_NEXT(nlMsg,len)){
        memset(rtInfo, 0, sizeof(struct route_info));
        parseRoutes(nlMsg, rtInfo,gateway);
    }

    free(rtInfo);
    close(sock);
    return 0;
}

int ping_status(char *ip)
{
    int i, status;
    pid_t pid;

    for (i = 0; i < 5; ++i) {
        if ((pid = vfork()) < 0) {
            printf("vfork error");
            continue;
        }

        char ip[256];
        if (get_gateway(ip) != 0){
            usleep(5*1000*1000);
            continue;
        }

        if (pid == 0) {
            if ( execlp("ping", "ping","-c","1",ip, (char*)0) < 0) {
                printf("execlp error\n");
                exit(1);
            }
        }

        waitpid(pid, &status, 0);
        if (status == 0)
            return 0;
    }
    return -1;
}
