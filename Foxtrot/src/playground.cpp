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

#include "foxtrot.h"


Playground::Playground(const char* name)
{
	this->name=name;
}

float Playground::getDistance(Robot* robot, GenericObject* object)
{
	; //TODO get the distance from robot to the object
	return 0.0;
}

void Playground::addRobot(Robot& robot)
{
	this->robots.push_back(robot);
}

void Playground::addWall(Wall& wall)
{
	this->walls.push_back(wall);
}

void Playground::addObstacle(Obstacle& obstacle)
{
	this->obstacles.push_back(obstacle);
}