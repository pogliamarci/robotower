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
#include "groundobjects.h"

Object::Objetc(int height, rgb_color color)
{
	this->height=height;
	this->color=color;
}

rgb_color Object::getColor()
{
	return this->color;
}

int Object::getHeight()
{
	return this->height;
}

Wall::Wall(int height, char color, Point start, Point end):Objetc(height, color)
{
	this->start=start;
	this->end=end;
}

Point Wall::getStart()
{
	return this->start;
}

Point Wall::getEnd()
{
	return this->end;
}

Tower::Tower(int height, char color, Point center, int radius):Object(height, color)
{
	this->center=center;
	this->radius=radius;
}

int Tower::getCenter()
{
	return this->center;
}

int Tower::getRadius()
{
	return this->radius;
}

Obstacle(int height, char color, Point a, Point b, Point c, Point d):Object(height, color)
{
	this->p[0]=a;
	this->p[1]=b;
	this->p[2]=c;
	this->p[3]=d;
}

Point getPoint(char n)
{
	return this->p[n];
}
