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
void imageMessageCallback(const sensor_msgs::CompressedImage::ConstPtr& message) {
    char c = '';
    /* decompress frame from spykee (JPEG encoded) */

    /* if Mat works, it should manage automatically memory management ...
    const CvMat compressed = cvMat(1, message->data.size(), CV_8UC1,
                                   const_cast<unsigned char*>(&message->data[0]));
    CvMat* raw_frame = cvDecodeImageM(&compressed, CV_LOAD_IMAGE_ANYCOLOR);
    */

    Mat compressed = Mat(1, message->data.size(), CV_8UC1,
                                   const_cast<unsigned char*>(&message->data[0]));
    Mat frame = imdecode(compressed, CV_LOAD_IMAGE_ANYCOLOR);

    /* maybe works... =): */
    // Mat frame = imdecode(message->data, CV_LOAD_IMAGE_ANYCOLOR);

    imshow("SpyKeeView", frame);
    c = waitKey(5);
    if (c == 'c') {
        exit(EXIT_SUCCESS);
    }
    // do what it is needed to do with the image...
}

//calcback per gestire le immagini ricevute
void rcvd_image(const  sensor_msgs::CompressedImage::ConstPtr& message)
{
	vector<unsigned char>::iterator i;
	vector<unsigned char> V=(vector<unsigned char>)message->data;
	ofstream img;
	img.open("img.jpg",  ios::out);
	for(i=V.begin(); i<V.end(); i++) img << *i;
	img.close();
}

//nodo di analisi immagini
int main (int argc, char** argv)
{
	ros::init(argc, argv, "vision");
	ros::NodeHandle ros_node;
	ros::Subscriber source = ros_node.subscribe("spykee_camera", 1, rcvd_image);
	ros::Publisher seen = ros_node.advertise<SpyKee::Vision>("chatter", 1000);
	namedWindow("SpyKeeView", CV_WINDOW_AUTOSIZE);
	Mat frame;

	/*// DEBUG - grab image from webcam
	CvCapture* capture;
	capture = cvCaptureFromCAM( CV_CAP_ANY );
	if (!capture) {
	    cout << "Error: cannot capture from camera" << endl;
	    return EXIT_FAILURE;
	}
	*/

	while (ros::ok())
	{
	    /*// DEBUG - grab image from webcam
        frame = cvQueryFrame(capture);
        */
	    Mat frame = imread("img.jpg");
        if (!frame.empty())
        {
            imshow("SpyKeeView", frame);
            int c = waitKey(5);
            if ((char) c == 'c') {
                break;
            }
        }
        ros::spinOnce();
    }
}
