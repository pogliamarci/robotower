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

class Robot
{
	public:
		Robot(const char* name);
		void setPosition(Point center, int angle);
		const char* getName();
		int setTanSpeed(int tanspeed);
		int setRotSpeed(int rotspeed);
		int getSonar(int direction);
		Point getPosition(char n);
		void updateStatus();
		
	private:
		const char* name;
		Point p[POINTNUMBERS];
		int sonardata[DIRECTIONS];
		int tanspeed;
		int rotspeed;
		bool seetower;
};