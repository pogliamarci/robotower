#include <iostream>
#include <vector>
#include "ros/ros.h"
#include "SpyKee/Motion.h"
#include "sensor_msgs/CompressedImage.h"

//calcback per gestire le immagini ricevute
rcvd_image()
{
	
}

//nodo si analisi immagini
int main (int argc, char** argv)
{
	ros::init(argc, argv, "vision");
	ros::NodeHandle ros_node;
	ros::Subscriber source = ros_node.subscribe("spykee_camera", 1, rcvd_image);
	ros::Publisher seen = ros_node.advertise<std_msgs::String>("chatter", 1000);
	while (ros::ok())
	{
		//pubblica le info prese dall'immagine precedente
		ros::spinOnce();
	}
}