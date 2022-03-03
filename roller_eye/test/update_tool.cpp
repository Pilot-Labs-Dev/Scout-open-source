

#include <fcntl.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <string>

#include<fstream>
#include<iostream>
#include<sstream>

typedef		unsigned short	    uint16;
typedef		unsigned int	      uint32;
typedef		unsigned char	    uint8;

#define VENDOR_REQ_TAG		0x56524551
#define VENDOR_READ_IO		_IOW('v', 0x01, unsigned int)
#define VENDOR_WRITE_IO		_IOW('v', 0x02, unsigned int)

#define VENDOR_ID_MAX	6
static char *vendor_id_table[] = {
	"VENDOR_SN_ID",
	"VENDOR_WIFI_MAC_ID",
	"VENDOR_LAN_MAC_ID",
	"VENDOR_BT_MAC_ID",
	"VENDOR_IMEI_ID",
	"VENDOR_P2P_UID_ID",
	"VENDOR_HW_VER_ID"
};

#define VENDOR_SN_ID		           1
#define VENDOR_WIFI_MAC_ID	  2
#define VENDOR_LAN_MAC_ID	  3
#define VENDOR_BT_MAC_ID	    4
#define VENDOR_IMEI_ID		          5
#define VENDOR_P2P_UID_ID	    6
#define VENDOR_HW_VER_ID		7
struct rk_vendor_req {
	uint32 tag;
	uint16 id;
	uint16 len;
	uint8 data[1024];
};

using namespace std;
string ReadParamFromVender(uint16 id)
{
    string value;

    if (id != VENDOR_SN_ID &&
        id != VENDOR_P2P_UID_ID &&
        id != VENDOR_HW_VER_ID) {
            return value;
    }

    int ret;
	uint8 buf[2048];
	struct rk_vendor_req *req;	
	req=(struct rk_vendor_req*)buf;
	int sys_fd=open("/dev/vendor_storage",O_RDWR,0);
	if(sys_fd<0){
		printf("vendor_storage open faileed\n");
		return value;
	}
	req->len    = 512;
    req->id      = id;
	req->tag   = VENDOR_REQ_TAG;
	ret              = ioctl(sys_fd,VENDOR_READ_IO,req);	
    if (0==ret && 
         req->len<128){
        req->data[req->len] = '\0';
        for (int i = 0; i < req->len; i++){
            printf("%c", req->data[i]);
        }
        printf("\n");
        value = string((char*)req->data);
    }

    close(sys_fd);

    return value;
}

void updateSn()
{
	 string sn    = ReadParamFromVender(VENDOR_SN_ID);    
	 ofstream file("/var/roller_eye/config/sn",ios::trunc);
     file<<sn;

	 printf("sn:%s\n", sn.c_str());
}

void updateP2p()
{
	 string p2p = ReadParamFromVender(VENDOR_P2P_UID_ID);    
 	 ofstream file("/var/roller_eye/config/p2p_uid",ios::trunc);
     file<<p2p;

	 printf("p2p:%s\n", p2p.c_str());
}

void updateHwver()
{
	 string hw  = ReadParamFromVender(VENDOR_HW_VER_ID);   
 	 ofstream file("/var/roller_eye/config/hw_ver",ios::trunc);
     file<<hw;

	 printf("hw:%s\n", hw.c_str());
}

int main(int argc, char *argv[])
{
	updateSn();
	updateHwver();
    return 0;
}


