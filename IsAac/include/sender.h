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

#include "ros/ros.h"
#include "SpyKee/Motion.h"
#include "IsAac/MediaVarianza.h"

class Sender 
{
	private:
		ros::Publisher motion;
		ros::Publisher debug_mediavarianza;
	public:
		Sender(ros::NodeHandle& n);
		void sendMotionMessage(int rot, int tan);
		void sendDebugMessage(float avg, float var, int time);
};

