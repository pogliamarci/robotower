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
#include <fstream>
#include <vector>
#include "ros/ros.h"
#include "SpyKee/Vision.h"
#include "sensor_msgs/CompressedImage.h"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace std;
using namespace cv;

// callback again, trying version without saving on file
void imageMessageCallback(const sensor_msgs::CompressedImage::ConstPtr& message)
{
    /*
     * i don't know why ROS decompresses images this way...
     *
    Mat compressed = Mat(1, message->data.size(), CV_8UC1,
                                   const_cast<unsigned char*>(&message->data[0]));
    Mat frame = imdecode(compressed, CV_LOAD_IMAGE_ANYCOLOR);
     */

    Mat frame = imdecode(message->data, CV_LOAD_IMAGE_ANYCOLOR);

    if (!frame.empty())
    {
        imshow("SpyKeeView", frame);
        char c = waitKey(5);
        if (c == 'c')
            exit(EXIT_SUCCESS);
    }
}

int main (int argc, char** argv)
{
	ros::init(argc, argv, "vision");
	ros::NodeHandle ros_node;
	ros::Subscriber source = ros_node.subscribe("spykee_camera", 1, imageMessageCallback);
	namedWindow("SpyKeeView", CV_WINDOW_AUTOSIZE);

	/* let's start it all */
	ros::spin();
}
