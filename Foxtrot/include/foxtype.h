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
#include <string>
#include <cmath>
#include <vector>

#define POINTNUMBERS 4
#define DIRECTIONS 4
#define ANGLEINCREMENT 45
#define ROBOTRADIUS 14.14213562

class Point
{
	public:
		Point(float x, float y);
		float x;
		float y;
		Point operator+ (Point p);
		Point& operator+= (const Point& p);
};

class Color_rgb
{
	public:
		Color_rgb(char r, char g, char b);
		char r;
		char g;
		char b;
		
};

