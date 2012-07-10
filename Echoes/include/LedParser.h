/*
 * RoboTower, Hi-CoRG based on ROS
 *
 * Copyright (C) 2012 Politecnico di Milano
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

#include <iostream>
#include "ReadSonar.h"
#include "ros/ros.h"
#include "Echoes/Led.h"

//classe per la gestione a basso livello dei led
class LedParser
{
	public:
		//costruttore
		LedParser(ReadSonar* read_sonar);
		bool ledCallback(Echoes::Led::Request& request, Echoes::Led::Response& response);
	private:
		//oggetto per comunicare via zigbee
		ReadSonar* sender;
		char buf[10];
		void sendCmd(char* buf);
};
