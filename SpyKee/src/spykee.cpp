#include <iostream>
#include "ros/ros.h"
#include "SpykeeManager.h"
#include "SpyKee/Motion.h"

#define SPYKEE_USER "admin"
#define SPYKEE_PWD "admin"

SpykeeManager* robot_ptr;
bool started;

void move(const SpyKee::Motion::ConstPtr& message)
{
	if (!started)
	{
		robot_ptr->unplug();
		started = true;
	}
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

	/* main loop */
	while (ros::ok())
	{
		// do something...
		ros::spinOnce();
	}
	delete robot_ptr;
	return EXIT_SUCCESS;
}
