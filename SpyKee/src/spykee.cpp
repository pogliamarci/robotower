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
	robot_ptr->move((int) message->leftTrack, (int) message->rightTrack);
}

int main(int argc, char** argv)
{
	started = false;

	/* ROS initialization */
	ros::init(argc, argv, "spykee");
	ros::NodeHandle ros_node;

	/* try to connect to the robot... */
	robot_ptr = new SpykeeManager(SPYKEE_USER, SPYKEE_PWD);

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
