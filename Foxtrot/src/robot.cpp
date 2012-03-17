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

#include "foxtrot.h"
#include "robot.h"

Robot::Robot(const char* name)
{
	this->name=name;
}

void Robot::setPosition(Point center, int angle)
{
	int i;
	for(i=0; i<POINTNUMBERS; i++)
	{
		this->p[i]+=Point(ROBOTRADIUS*cos(ANGLEINCREMENT*(i+1)+angle), ROBOTRADIUS*sin(ANGLEINCREMENT*(i+1)+angle));
	}
}

const char* Robot::getName()
{
	return this->name;
}

int Robot::setTanSpeed(int tanspeed)
{
	this->tanspeed=tanspeed;
}

int Robot::setRotSpeed(int rotspeed)
{
	this->rotspeed=rotspeed;
}

int Robot::getSonar(int direction)
{
	if(direction<DIRECTIONS)
		return this->sonardata[direction];
	else return -1; //TODO emit exception
}

Point Robot::getPosition(char n)
{
	if(n<POINTNUMBERS)
		return p[n];
	else return p[0]; //TODO emit exception
}

void Robot::updateStatus()
{
	; //TODO update state
}
