/*
 * RoboTower, Hi-CoRG based on ROS
 *
 *
 * Copyright (C) 2011 Marcello Pogliani, Davide Tateo
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

#include "ReadSonar.h"
#include "sstream"

//#define DEBUG
#define BAUDRATE 	B19200
#define TOUT		300 //msec
#define MAX_C_RECV 	40
#define CHAR_PAUSE 	3000 //usec
#define MAX_BUF_CHAR	32767
	
using namespace std;

ReadSonar::ReadSonar(std::string sdev,float to_meter) throw (ReadSonarDeviceException)
:ReadSonarBase(to_meter){

	fd = open(sdev.c_str(), O_RDWR | O_NOCTTY );
	if(fd>=0){
		tcgetattr(fd,&oldtio);
		bzero(&newtio, sizeof(termios));
		newtio.c_cflag = BAUDRATE  | CS8 | CLOCAL | CREAD | CLOCAL ;
		newtio.c_iflag = IGNPAR;
		newtio.c_oflag = 0;
		newtio.c_lflag &= ~ICANON;
			 
		set_fd(fd);
		
		newtio.c_cc[VTIME]    = 0;
		newtio.c_cc[VMIN]     = 1;   
		tcflush(fd, TCIFLUSH);
		tcsetattr(fd,TCSANOW,&newtio);
	
		buffer=new CharCircularBuffer(MAX_BUF_CHAR,'\r');
				
	}
	else{
		throw ReadSonarDeviceException();
	}	
}

ReadSonar::~ReadSonar(){
    if(fd>=0) {
        sendStop();
        close(fd);
    }
	if(buffer) {
	    delete buffer;
	}
}

bool ReadSonar::isReady(){
	return (fd>=0);
}

int ReadSonar::readData(){
	if(fd<0) {
		return -1;
	}

	int res;
	unsigned int count_c=0;
	
	#ifdef DEBUG 
	printf("ciclo read\n");
	#endif
	res=0;

	do {
		switch(waitData(TOUT)){
			case SerialCommunication::wait_ok:
				res = read(fd,tmp_buf,max_buf_tmp);
				if(res<=0){
					fprintf(stderr,"(1) Sonar READ Error on fileno %d\n",fd);
					res=-1;
				}
				if(buffer->addNChar(tmp_buf,res)!=res){
					count_c	= count_c+res;			
					fprintf(stderr,"(1) Sonar BUF_FULL on fileno %d\n",fd);
					res=-1;
				}
				break;
			case SerialCommunication::wait_err: 
				fprintf(stderr,"(1) Sonar Error on fileno %d\n",fd);
				res=-1;
				break;
			case SerialCommunication::wait_tout: 
				fprintf(stderr,"(1) Sonar TOUT on fileno %d\n",fd);
				res=-1;
				break;
		}		
		
		
		if(res==-1)break;
		
	}while(count_c<MAX_C_RECV && buffer->getLineCount()<=0);
	
	if(res<=0)return -1;
	return 0;
}

int ReadSonar::sendRun(){
	tcflush(fd, TCIFLUSH);
	return sendStringCommand((char *)"R\r",2);
}

int ReadSonar::sendStop(){
	return sendStringCommand((char *)"S\r", 2);
}

int ReadSonar::sendStringCommand(char *cmd,int len){
	if(fd<0)return -1;
	for(int i=0;i<len;i++){
		if(write(fd,&cmd[i],1)!=1)return -1;
		usleep(CHAR_PAUSE);
	}
	return 0;
}
