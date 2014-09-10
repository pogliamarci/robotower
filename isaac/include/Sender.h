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

class Sender 
{
	private:
		ros::Publisher motion;
		ros::ServiceClient yellowled;
		ros::ServiceClient greenled;
		bool yellowBlinking;
		bool greenBlinks;
		bool ledEnabled;
		void sendYellow();
		void sendGreen();
	public:
		Sender(ros::NodeHandle& n);
		void sendMotionMessage(int rot, int tan);
		void setLed(bool isTrapped, bool seenSomething);
		void disableLed();
};

