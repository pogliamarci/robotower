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

#include "RosPublisher.h"

#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

RosPublisher::RosPublisher()
{
	hasToQuit = false;
	enableIsaacPublisher = n.advertise<std_msgs::Bool>("isac_enable", 1000);
	enableCardPublisher = n.advertise<std_msgs::String>("rfid_enable", 1000);
	resetRobotPublisher = n.advertise<std_msgs::Bool>("echoes_reset", 1000);
}

void RosPublisher::run()
{
	ros::Rate rate(15);
	while(ros::ok() && !hasToQuit)
	{
		ros::spinOnce();
		rate.sleep();
	}
	if(!hasToQuit)
	{
		emit rosQuits();
	}
}
 
void RosPublisher::quitNow()
{
	hasToQuit = true;
}
 
 
void RosPublisher::resetRobot()
{
	std_msgs::Bool message;
	message.data = true;
	enableIsaacPublisher.publish(message);
}

void RosPublisher::enableRFID(std::string id)
{
	std_msgs::String message;
	message.data = id;
	enableCardPublisher.publish(message);
}

void RosPublisher::enableIsaac(bool isEnabled)
{
	std_msgs::Bool message;
	message.data = isEnabled;
	enableIsaacPublisher.publish(message);
}
