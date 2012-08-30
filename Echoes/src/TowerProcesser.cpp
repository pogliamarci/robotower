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

#include "TowerProcesser.h"
#include <iostream>
#include <cstdlib>
#include "Echoes/Towers.h"

TowerProcesser::TowerProcesser(ros::Publisher pub)
{
	publisher = pub;
}

// FIXME manca la gestione degli errori
void TowerProcesser::process(string str)
{
	const string destroyed = "destroyed ";
	int index = str.find(destroyed) + destroyed.length();
	int tower = atoi(str.substr(index).c_str());
	/* publish the message to ROS */
	Echoes::Towers msg;
	msg.towerId = tower;
	publisher.publish(msg);
}
