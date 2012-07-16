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

#include "RfidProcesser.h"

#include<iostream>
#include<cstdio>


using namespace std;


RfidProcesser::RfidProcesser(ros::Publisher pub)
{
	publisher = pub;
}

void RfidProcesser::process(string str)
{
	/* catch the data */
	string data = str.substr(0, 10);

	/* check the checksum */
	uint8_t computedChecksum = checksum(data.c_str(), 10);
	uint8_t providedChecksum = decodeByte(str.at(10), str.at(11));
	if(computedChecksum != providedChecksum)
	{
		cerr << "Checksum FAILED" << endl;
	}
	else
	{
		/* and finally, send data as a ROS message (if successful) */
		Echoes::Rfid msg;
		msg.id = data;
		publisher.publish(msg);
	}
}

RfidProcesser::~RfidProcesser()
{
	// TODO Auto-generated destructor stub
}

uint8_t RfidProcesser::checksum(const char* in, size_t len)
{
	uint8_t checksum = 0;
	for(unsigned int i = 0; i < len; i += 2) {
		checksum ^= decodeByte(in[i], in[i+1]);
	}
	return checksum;
}

uint8_t RfidProcesser::decodeByte(char msb, char lsb)
{
	unsigned int n = msb > '9' ? msb - 'A' + 10 : msb - '0';
	unsigned int m = lsb > '9' ? lsb - 'A' + 10 : lsb - '0';
	return (n << 4) | m;
}
