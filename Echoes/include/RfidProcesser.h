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

#ifndef RFIDPROCESSER_H_
#define RFIDPROCESSER_H_

#include "Processer.h"

#include <stdint.h>
#include "ros/ros.h"
#include "Echoes/Rfid.h"

class RfidProcesser : public Processer
{
	public:
		RfidProcesser(ros::Publisher pub);
		void process(string str);
	private:
		uint8_t checksum(const char* in, size_t len);
		uint8_t decodeByte(char msb, char lsb);
		ros::Publisher publisher;

};

#endif /* RFIDPROCESSER_H_ */
