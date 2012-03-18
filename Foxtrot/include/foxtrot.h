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

class GenericObject
{
	public:
		GenericObject(int height, Color_rgb* color);
		Color_rgb getColor();
		int getHeight();
	private:
		int height;
		Color_rgb* color;
};

class Wall : public GenericObject
{
	public:
		Wall(int height, Color_rgb* color, Point* start, Point* end);
		Point getStart();
		Point getEnd();
	private:
		Point* start;
		Point* end;
};

class Tower : public GenericObject
{
	public:
		Tower(int height, Color_rgb* color, Point* center, int radius);
		Point getCenter();
		int getRadius();
	private:
		Point* center;
		int radius;
};

class Obstacle : public GenericObject
{
	public:
		Obstacle(int height, Color_rgb* color, Point* a, Point* b, Point* c, Point* d);
		Point getPoint(int n);
	private:
		Point* p[POINTNUMBERS];
};

class Robot
{
	public:
		Robot(const char* name);
		void setPosition(Point center, int angle);
		const char* getName();
		void setTanSpeed(int tanspeed);
		void setRotSpeed(int rotspeed);
		int getSonar(int direction);
		Point getPosition(int n);
		void updateStatus();
		
	private:
		const char* name;
		Point* p[POINTNUMBERS];
		int sonardata[DIRECTIONS];
		int tanspeed;
		int rotspeed;
		bool seetower;
};

class Playground
{
	public:
		Playground(const char* name);
		float getDistance(Robot* robot, GenericObject* object);
		void addRobot(Robot& robot);
		void addWall(Wall& wall);
		void addObstacle(Obstacle& obstacle);
	private:
		const char* name;
		std::vector<Wall> walls;
		std::vector<Obstacle> obstacles;
		std::vector<Robot> robots;
	
};

