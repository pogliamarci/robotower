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
#include "spykee/Motion.h"
#include "sensor_msgs/CompressedImage.h"
#include "std_msgs/Int8.h"

#define SPYKEE_USER "admin"
#define SPYKEE_PWD "admin"

SpykeeManager* spykeeManager;

void move(const spykee::Motion::ConstPtr& message)
{
	int right_track = message->tanSpeed - message->rotSpeed;
	int left_track = message->tanSpeed + message->rotSpeed;
	spykeeManager->move(left_track, right_track);
}

void publishFrame(ros::Publisher& img_pub, vector<unsigned char>* image)
{
	sensor_msgs::CompressedImage image_msg;
	image_msg.data = *image;
	image_msg.format = "jpeg";
	img_pub.publish(image_msg);
	delete image;
}

void publishBattery(ros::Publisher& bat_pub, int level)
{
	std_msgs::Int8 message;
	message.data = level;
	bat_pub.publish(message);
}

int main(int argc, char** argv)
{
	bool connectionSuccessful = false;
	while(!connectionSuccessful)
	{
		try
		{
			spykeeManager = new SpykeeManager(SPYKEE_USER, SPYKEE_PWD);
			connectionSuccessful = true;
		}
		catch (SpykeeException& e)
		{
			cerr << "Error connecting with the robot" << endl;
			cerr << "Make sure that Spykee is turned on and " <<
					"that your wireless network is working! " <<
					"(Retrying in 5 seconds...)" << endl;
			sleep(5);
		}
	}

	/* NOTE: node is initialize here so that SIGINT can kill the application during
	 * the previous cycle (ROS install a signal handler for SIGINT, that should make ros::ok()
	 * return false, but that didn't worked well when inserted in the previous cycle...
	 */
	ros::init(argc, argv, "spykee");
	ros::NodeHandle ros_node;
	ros::Subscriber sub = ros_node.subscribe("spykee_motion", 1000, move);
	ros::Publisher img_pub = ros_node.advertise<sensor_msgs::CompressedImage>("spykee_camera", 3);
	ros::Publisher bat_pub = ros_node.advertise<std_msgs::Int8>("spykee_battery", 3);

	spykeeManager->unplug();
	spykeeManager->setLed(0, false); /* try to turn off the flash */
	spykeeManager->setCameraStatus(true);

	ros::AsyncSpinner spinner(1);
	spinner.start();
	while (ros::ok())
	{
		spykeeManager->readPacket();
		if(spykeeManager->hasImage())
		{
			publishFrame(img_pub, spykeeManager->readImage());
		}
		if(spykeeManager->hasBattery())
		{
			publishBattery(bat_pub, spykeeManager->getBatteryLevel());
		}
	}
	ros::waitForShutdown();
	delete spykeeManager;
	return EXIT_SUCCESS;
}
