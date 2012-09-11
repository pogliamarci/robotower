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

SpykeeManager* spykee;

void move(const SpyKee::Motion::ConstPtr& message)
{
	int right_track = message->tanSpeed - message->rotSpeed;
	int left_track = message->tanSpeed + message->rotSpeed;
	spykee->move(left_track, right_track);
}

void publishFrame(ros::Publisher& img_pub)
{
	vector<unsigned char>* image = spykee->getImage();

	sensor_msgs::CompressedImage image_msg;
	image_msg.data = *image;
	image_msg.format = "jpeg";
	img_pub.publish(image_msg);
	delete image;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "spykee");
	ros::NodeHandle ros_node;

	bool connectionSuccessful = false;
	while(!connectionSuccessful)
	{
		try
		{
			spykee = new SpykeeManager(SPYKEE_USER, SPYKEE_PWD);
			connectionSuccessful = true;
		}
		catch (SpykeeException& e)
		{
			cerr << "Error connecting with the robot" << endl;
			cerr << "Make sure that Spykee is turned on and " <<
					"that your wireless network is working! " <<
					"(Retrying in 7 seconds...)" << endl;
			sleep(7);
		}
	}

	ros::Subscriber sub = ros_node.subscribe("spykee_motion", 1000, move);
	ros::Publisher img_pub = ros_node.advertise<sensor_msgs::CompressedImage>("spykee_camera", 3);

	spykee->unplug();
	spykee->startCamera();

	while (ros::ok())
	{
		publishFrame(img_pub);
		ros::spinOnce();
	}
	delete spykee;
	return EXIT_SUCCESS;
}
