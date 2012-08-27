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

#include "ReadSonar.h"
#include "sstream"
#include <iostream>

#define BAUDRATE 		B19200
#define TOUT			300 //msec
#define MAX_C_RECV 		40
#define CHAR_PAUSE 		3000 //usec
#define MAX_BUF_CHAR	32767

using namespace std;

const char* ReadSonarDeviceException::what() const throw()
{
  return "Sonar port open error!";
}

ReadSonar::ReadSonar(std::string serialDevice) throw (ReadSonarDeviceException)
{
	pthread_mutex_init(&mutex, NULL);
	buffer = NULL; /* wtf? */
	tmp_buf=NULL;
	tmp_buf=new char[MAX_TMP_BUF];
	fd = open(serialDevice.c_str(), O_RDWR | O_NOCTTY );
	if(fd>=0)
	{
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
	
		buffer = new CharCircularBuffer(MAX_BUF_CHAR,'\r');
				
	}
	else
	{
		throw ReadSonarDeviceException();
	}
}

ReadSonar::~ReadSonar()
{
	if(fd>=0) 
	{
		close(fd);
	}
	if(buffer)
	{
		delete buffer;
	}
	if(tmp_buf)
	{
		delete [] tmp_buf;
	}
	pthread_mutex_destroy(&mutex);
}

bool ReadSonar::isReady(){
	return (fd>=0);
}

int ReadSonar::readData()
{
	if(fd<0) 
	{
		return -1;
	}

	int res;
	unsigned int count_c=0;
	
	res=0;

	do {
		switch(waitData(TOUT))
		{
			case SerialCommunication::wait_ok:
				res = read(fd,tmp_buf,MAX_TMP_BUF);
				if(res<=0)
				{
					cerr << "(1) Sonar READ Error on fileno " << fd << endl;
					res=-1;
				}
				if(buffer->addNChar(tmp_buf,res)!=res)
				{
					count_c	= count_c+res;			
					cerr << "(1) Sonar BUF_FULL on fileno " << fd << endl;
					res=-1;
				}
				break;
			case SerialCommunication::wait_err: 
				cerr << "(1) Sonar Error on fileno " << fd << endl;
				res=-1;
				break;
			case SerialCommunication::wait_tout: 
				cerr << "(1) Sonar TOUT on fileno " << fd << endl;
				res=-1;
				break;
		}
		
		if(res==-1)break;
		
	}while(count_c<MAX_C_RECV && buffer->getLineCount()<=0);
	
	if(res<=0)return -1;
	return 0;
}

std::string ReadSonar::getLine()
{
	if(buffer->getLineCount()<=0) return "Error";
	int len=buffer->removeLine(tmp_buf,MAX_TMP_BUF);
	if(len<=0)return "Error";
	if(tmp_buf[len-1]=='\n')tmp_buf[len-1]='\0';
	return tmp_buf;
}

unsigned int ReadSonar::getLineToParseNum()
{
	return buffer->getLineCount();
}

int ReadSonar::sendStringCommand(char *cmd,int len)
{
	pthread_mutex_lock(&mutex);
	if(fd<0)return -1;
	for(int i=0;i<len;i++)
	{
		if(write(fd,&cmd[i],1)!=1)return -1;
		usleep(CHAR_PAUSE);
	}
	pthread_mutex_unlock(&mutex);
	return 0;
}
