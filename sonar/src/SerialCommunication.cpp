/*
 * RoboTower, Hi-CoRG based on ROS
 *
 * Copyright (C) 2012 Politecnico di Milano
 * Copyright (C) 2012 Marcello Pogliani, Davide Tateo
 * Versione 1.0
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation, version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

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
