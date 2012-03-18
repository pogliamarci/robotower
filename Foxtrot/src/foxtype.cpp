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

#include "foxtype.h"

Point::Point(float x, float y)
{
	this->x=x;
	this->y=y;
}

Point Point::operator+ (Point p)
{
	return Point(this->x+p.x, this->y+p.y);
}

Point& Point::operator+=(const Point& p)
{
            x += p.x ;
            y += p.y ;
            return *this ;
}

Color_rgb::Color_rgb(char r, char g, char b)
{
	this->r=r;
	this->g=g;
	this->b=b;
}
