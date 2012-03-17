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

#include "playground.h"

Playground::Playground(const char* name)
{
	this->name=name;
}

float Playground::getDistance(Robot* robot, Object* object)
{
	; //TODO get the distance from robot to the object
}

void Playground::addRobot(Robot* robot)
{
	this->robots.insert(robot);
}

void Playground::addWall(Wall* wall)
{
	this->walls.insert(wall);
}

void Playground::addObstacle(Obstalce* obstacle)
{
	this->obstacles.insert(obstacle);
}