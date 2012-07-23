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

#include "SonarBuffer.h"
#include "IsaacStrategy.h"
#include "SensorStatus.h"
#include "Sender.h"

int main(int argc, char** argv)
{
	srand((unsigned)time(NULL));
	
	//sonar variable
	SonarBuffer sonarBuffer;

	//data
	SensorStatus sensors;
	
	//reasoning Strategy
	IsaacStrategy isaacStrategy;
	
	//ros initialization
	ros::init(argc, argv, "isaac");
	ros::NodeHandle ros_node = ros::NodeHandle();
	ros::Subscriber sonar_sub = ros_node.subscribe("sonar_data", 1,
			&SensorStatus::fromSonarCallback, &sensors);
	ros::Subscriber vision_sub = ros_node.subscribe("vision_results", 1,
				&SensorStatus::fromVisionCallback, &sensors);
	ros::ServiceClient client = ros_node.serviceClient<Echoes::Led>("led_data");

	ros::Rate loop_rate(LOOPRATE);

	Sender message_sender(ros_node);

	while (ros::ok())
	{
		sonarBuffer.insert(sensors.getSonar(NORTH));
		sonarBuffer.setTempoBloccato();
		
		isaacStrategy.activateStrategy(sensors, sonarBuffer.getTempoBloccato());
		
		message_sender.sendMotionMessage(isaacStrategy.getTanSpeed(), isaacStrategy.getRotSpeed());
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
