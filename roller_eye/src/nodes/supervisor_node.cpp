#include <sys/stat.h>
#include <sys/wait.h>
#include <fcntl.h> 
#include <unistd.h>
#include <syslog.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <thread>

void watchdog()
{
    int fd = open("/dev/watchdog", O_WRONLY); 
    int ret = 0;
    if (fd == -1) {
        perror("watchdog");
        //exit(EXIT_FAILURE);
    }
    while (1) {
        if (fd>=0){
            ret = write(fd, "\0", 1);
            if (ret != 1) {
                ret = -1;
                break;
            }
        }
        sleep(2);
        system("sudo  cleanLog.sh");
    }
}

int main(int argc, char **argv)
{     
    watchdog(); 
    return 0;
}
