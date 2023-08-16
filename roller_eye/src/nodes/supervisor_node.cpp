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

void superviosor_rkisp_3A()
{
    char* const argv_execv[]={"rkisp_3A_server","--mmedia=/dev/media0",NULL};
    while(1){   
		int status;
        pid_t pid = fork(); 
        if (pid == -1) {
            fprintf(stderr, "fork() error.errno:%d error:%sn", errno, strerror(errno));
            break;
        }
        if (pid == 0) {
            execv("/usr/bin/rkisp_3A_server",  argv_execv);
        }
 
        if (pid > 0) {
            pid = wait(&status); 
            fprintf(stdout, "wait return\n");
        } 
    }
}

int main(int argc, char **argv)
{     
    watchdog(); 
    return 0;
}
