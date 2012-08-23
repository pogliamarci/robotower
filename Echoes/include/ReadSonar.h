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

#ifndef READOSONAR_H_
#define READOSONAR_H_

#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <termios.h>
#include <exception>
#include <vector>
#include <string>
#include "SerialCommunication.h"
#include "CharCircularBuffer.h"

class ReadSonarDeviceException: public std::exception
{
	public:
		virtual const char* what() const throw();
};

class ReadSonar : public SerialCommunication
{
	public:
		ReadSonar(std::string serialDevice) throw (ReadSonarDeviceException);
		int sendStringCommand(char *cmd,int len);
		std::string getLine();
		bool isReady();
		int readData();
		unsigned int getLineToParseNum();
		~ReadSonar();
	private:
		int fd;  	//descrittore del file per leggere/scrivere sulla seriale
		termios oldtio,newtio;
		CharCircularBuffer * buffer;
		char * tmp_buf;
		static const int MAX_TMP_BUF = 256;
		pthread_mutex_t mutex;
};

#endif
