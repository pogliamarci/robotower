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
#include <vector>
#include "ros/ros.h"
#include "SpykeeManager.h"
#include "SpyKee/Motion.h"
#include "sensor_msgs/CompressedImage.h"

#define SPYKEE_USER "admin"
#define SPYKEE_PWD "admin"

SpykeeManager* robot_ptr;
bool started;

void move(const SpyKee::Motion::ConstPtr& message)
{
	int right_track = message->tanSpeed - message->rotSpeed;
	int left_track = message->tanSpeed + message->rotSpeed;
	robot_ptr->move(left_track, right_track);
}

int main(int argc, char** argv)
{
	started = false;

	/* ROS initialization */
	ros::init(argc, argv, "spykee");
	ros::NodeHandle ros_node;

	/* try to connect to the robot... */
	char user[] = SPYKEE_USER;
	char pwd[] = SPYKEE_PWD;
	try {
		robot_ptr = new SpykeeManager(user, pwd);
	} catch (exception& e) {
		cerr << "Error connecting with the robot" << endl;
		cerr << "Make sure that Spykee is turned on and " <<
				"that your wireless network is working" << endl;
		exit(EXIT_FAILURE);
	}

	/* subscribe to motion messages */
	ros::Subscriber sub = ros_node.subscribe("spykee_motion", 1000, move);
	ros::Publisher img_pub = ros_node.advertise<sensor_msgs::CompressedImage>("spykee_camera", 3);

	/* start camera... */
	robot_ptr->unplug();
	robot_ptr->startCamera();

	while (ros::ok())
	{
		vector<unsigned char>* image = robot_ptr->getImage();
		sensor_msgs::CompressedImage image_msg;
		image_msg.data = *image;
		image_msg.format = "jpeg";

		img_pub.publish(image_msg);

		ros::spinOnce();
		delete image;
	}
	delete robot_ptr;
	return EXIT_SUCCESS;
}
