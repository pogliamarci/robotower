#include "SerialCommunication.h"
#include "sys/ioctl.h"
#include <sched.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <unistd.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <strings.h> 

int SerialCommunication::waitData(int msec_tout){
	bool rexec;
	int v;
	int fd=ufd[0].fd;
	
	do{
		rexec=false;
		v = poll (ufd, 1, msec_tout);
		if (v < 0){
			if(errno==EINTR){
				fprintf(stderr,"poll EINTR on file %d\n",fd);
				rexec=true;
			}
			else {
				fprintf(stderr, "poll error on file %d,errno=%d\n",fd,errno);
				return wait_err;
			}
		}
		else if (v == 0){
			//fprintf(stderr,"time out on read on file %d\n",fd);
			return wait_tout;
		}
	}while(rexec);
	return wait_ok;
}

void SerialCommunication::set_fd(int fd){
	ufd[0].fd = fd;
	ufd[0].events = POLLIN;		
}
/*int SerialCommunication::set_low_latency(){
	int fd=ufd[0].fd;

   struct serial_struct serial;
   int result;
   result=ioctl(fd, TIOCGSERIAL, &serial);

   if (result) {
		   return result;
   } else {
		   serial.flags |= ASYNC_LOW_LATENCY;
		   serial.xmit_fifo_size = 1;
		   ioctl(fd, TIOCSSERIAL, &serial);
		   if (result) {
				   return result;
		   }
   }
   return result;

	
}*/
