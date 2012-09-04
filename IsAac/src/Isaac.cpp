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

#include <iostream>

#include "brian.h"

#include "ros/ros.h"
#include "std_msgs/Bool.h"

#include "IsaacStrategy.h"
#include "SensorStatus.h"
#include "Sender.h"

bool isEnabled = false;

void enabler(const std_msgs::Bool& msg)
{
	isEnabled = msg.data;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "isaac");
	ros::NodeHandle ros_node = ros::NodeHandle();
	ros::Rate looprate(LOOPRATE);

	SensorStatus sensors; //data
	IsaacStrategy isaacStrategy; //reasoning Strategy
	Sender sender(ros_node);

	ros::Subscriber sonar_sub = ros_node.subscribe("sonar_data", 1,
			&SensorStatus::fromSonarCallback, &sensors);
	ros::Subscriber vision_sub = ros_node.subscribe("vision_results", 1,
			&SensorStatus::fromVisionCallback, &sensors);
	ros::Subscriber disablerfid_sub = ros_node.subscribe("rfid_action", 1,
			&SensorStatus::rfidActionCallback, &sensors);
	ros::Subscriber enable_sub = ros_node.subscribe("isaac_enable", 1, enabler);

	while (ros::ok())
	{
		if (isEnabled)
		{
			isaacStrategy.activateStrategy(sensors);
			sender.sendMotionMessage(isaacStrategy.getTanSpeed(),
					isaacStrategy.getRotSpeed());
			sender.setLed(isaacStrategy.isTrapped(),
					isaacStrategy.hasSeenSomething());
		} else {
			sender.disableLed();
		}
		ros::spinOnce();
		looprate.sleep();
	}
	return 0;
}
