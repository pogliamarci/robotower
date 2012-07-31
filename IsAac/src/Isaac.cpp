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
#include <string>
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
	//ros initialization
	ros::init(argc, argv, "isaac");
	ros::NodeHandle ros_node = ros::NodeHandle();

	srand((unsigned)time(NULL));

	//data
	const char* filename = "../../rfidconfig.txt";
	SensorStatus sensors(filename);
	
	//reasoning Strategy
	IsaacStrategy isaacStrategy;
	
	// definition of messages\services handlers
	ros::Subscriber enable_sub = ros_node.subscribe("isaac_enable", 1, enabler);
	ros::Subscriber sonar_sub = ros_node.subscribe("sonar_data", 1,
			&SensorStatus::fromSonarCallback, &sensors);
	ros::Subscriber vision_sub = ros_node.subscribe("vision_results", 1,
				&SensorStatus::fromVisionCallback, &sensors);
	ros::Subscriber rfid_sub = ros_node.subscribe("rfid_data", 1,
					&SensorStatus::fromRfidCallback, &sensors);
	ros::Subscriber disablerfid_sub = ros_node.subscribe("rfid_disable", 1,
			&SensorStatus::enableRfidCallback, &sensors);
	ros::ServiceClient client = ros_node.serviceClient<Echoes::Led>("led_data");

	ros::Rate loop_rate(LOOPRATE);

	Sender message_sender(ros_node);

	cout << "Isaac OK. Waiting for the enable message" << endl;

	while (ros::ok())
	{
		if(isEnabled)
		{
			isaacStrategy.activateStrategy(sensors);
			message_sender.sendMotionMessage(isaacStrategy.getTanSpeed(),
					isaacStrategy.getRotSpeed());
		}
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
