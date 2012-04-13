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

#include "sensorstatus.h"

SensorStatus::SensorStatus()
{
	for (int i=0; i < CARDINAL_POINTS; i++)
		sonar[i] = 0;
	tower_found = false;
	tower_position = 0;
	factory_found = false;
	factory_position = 0;
}

void SensorStatus::fromSonarCallback(const Echoes::Sonar& message)
{
	sonar[NORTH] = message.north;
	sonar[SOUTH] = message.south;
	sonar[EAST] = message.east;
	sonar[WEST] = message.west;
}

void SensorStatus::fromVisionCallback(const Vision::Results& message)
{
	tower_found = message.towerFound;
	tower_position = message.towerPos;
	factory_found = message.factoryFound;
	factory_position = message.factoryPos;
}
