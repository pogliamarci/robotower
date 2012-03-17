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

class GenericObject
{
	public:
		Objetc(int height, char* color);
		rgb_color getColor();
		int getHeight;
	private:
		int height;
		Color_rgb color;
};

class Wall : public GenericObject
{
	public:
		Wall(int height, char color, Point start, Point end);
		Point getStart();
		Point getEnd();
	private:
		Point start;
		Point end;
};

class Tower : public GenericObject
{
	public:
		Tower(int height, char color, Point center, int radius);
		int getCenter();
		int getRadius();
	private:
		Point center;
		int radius
};

class Obstacle : public GenericObject
{
	public:
		Obstacle(int height, char color, Point a, Point b, Point c, Point d);
		Point getPoint(char n);
	private:
		Point p[POINTNUMBERS];
};

