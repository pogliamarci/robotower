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

#include "ReadSonarBase.h"

class ReadSonar : public SerialCommunication, public ReadSonarBase
{
	int fd;  	//descrittore del file per leggere/scrivere sulla seriale
	termios oldtio,newtio;

	public:
		int sendStringCommand(char *cmd,int len);
	public:
		ReadSonar(std::string sdev,float to_meter)
		throw (ReadSonarDeviceException);
		~ReadSonar();

		virtual bool isReady();
		virtual int readData();
};

#endif
