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

GenericObject::GenericObject(int height, Color_rgb* color)
{
	this->height=height;
	this->color=color;
}

Color_rgb GenericObject::getColor()
{
	return *this->color;
}

int GenericObject::getHeight()
{
	return this->height;
}

Wall::Wall(int height, Color_rgb* color, Point* start, Point* end):GenericObject(height, color)
{
	this->start=start;
	this->end=end;
}

Point Wall::getStart()
{
	return *this->start;
}

Point Wall::getEnd()
{
	return *this->end;
}

Tower::Tower(int height, Color_rgb* color, Point* center, int radius):GenericObject(height, color)
{
	this->center=center;
	this->radius=radius;
}

Point Tower::getCenter()
{
	return *this->center;
}

int Tower::getRadius()
{
	return this->radius;
}

Obstacle::Obstacle(int height, Color_rgb* color, Point* a, Point* b, Point* c, Point* d):GenericObject(height, color)
{
	this->p[0]=a;
	this->p[1]=b;
	this->p[2]=c;
	this->p[3]=d;
}

Point Obstacle::getPoint(int n)
{
	return *this->p[n];
}
