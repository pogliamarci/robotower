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
#include "ros/ros.h"
#include "sensor_msgs/CompressedImage.h"
#include "Vision/Results.h"

#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "ImageAnalyzer.h"

using namespace std;
using namespace cv;

class VisionParameters {
	private:
		KnnColorClassifier cc;
		PixelMap pm;
		ColorDataset cd;
		ImageAnalyzer analyzer;
		char* dataset;
		char* classifier;
	public:
		VisionParameters() : analyzer(&cc, &pm, &cd) {
			/* default file paths */
			char defaultDataset[] = "DataSet.dts";
			char defaultClassifier[] = "classifier[5-17].kcc";
			dataset = defaultDataset;
			classifier = defaultClassifier;
		}
		void buildClassifier() {
			cd.load(dataset);
			cout << "----> Color Data set  [OK]" << "caricato file:" << dataset << endl;
			//Creazione del classificatore colore
			cc.fast_build(&cd, 5, 17, false);
			cc.save_matrix(classifier);
		}
		void loadClassifier() {
			cout << "----> No classifier loaded, loading default classifier" << endl;
			cc.load_matrix(classifier);
			cout << "----> Classifier loaded  [OK]"<<endl;
		}
		Vision::Results startAnalysis(Mat& frame) {
			return analyzer.analyze(frame);
		}
		void setDataset(char* dataset) {
			this->dataset = dataset;
		}
		void setClassifier(char* classifier) {
			this->classifier = classifier;
		}
};

ros::Publisher results_publisher;
VisionParameters* vision;

void imageAction(Mat& frame)
{
	if (frame.empty()) return;
	results_publisher.publish(vision->startAnalysis(frame));
	imshow("SpyKeeView", frame);
	char c = waitKey(10);
	if(c == 'c' || c == 27) /* 27 is the ESC character ASCII code */
	{
		exit(EXIT_SUCCESS);
	}
}

void imageMessageCallback(const sensor_msgs::CompressedImage::ConstPtr& message)
{
	cv::Mat f = imdecode(message->data, CV_LOAD_IMAGE_ANYCOLOR);
	imageAction(f);
}

int main (int argc, char** argv)
{
	/* variables to read from file the frame to analyze */
	bool from_file = false;
	Mat frameFromFile;

	/* ROS variables */
	ros::init(argc, argv, "vision");
	ros::NodeHandle ros_node;
	ros::Subscriber source = ros_node.subscribe("spykee_camera", 1, imageMessageCallback);
	results_publisher = ros_node.advertise<Vision::Results>("vision_results", 10);

	/* OpenCV init */
	namedWindow("SpyKeeView", CV_WINDOW_AUTOSIZE);

	/* Command line argument parsing */
	for(int i=1; i<argc; i++)
	{
		if(strcmp(argv[i],"-h") == 0)
		{
		   cout << "Utilizzo:\n"
				<< "-h\t\t visualizza questo messaggio\n"
				<< "-l\t\t Carica il file DataSet.dts e genera il file .kcc\n"
				<< "-L [filename]\t carica il Color Data Set da file e genera il file .kcc\n"
				<< "-k [filename]\t carica il file .kcc da file\n"
				<< "-f [filename]\t carica il frame da analizzare da file\n"
				<< endl;
			exit(EXIT_SUCCESS);
		}
		else if((strcmp(argv[i], "-l") == 0))
		{
			vision->buildClassifier();
		}
		else if(strcmp(argv[i],"-L" ) == 0 && argc > (i+1))
		{
			vision->setDataset(argv[++i]);
			vision->buildClassifier();
		}
		else if((strcmp(argv[i], "-k") == 0) && (argc > (i+1)))
		{
			vision->setClassifier(argv[++i]);
		}
		else if((strcmp(argv[i],"-f") == 0) && (argc > (i+1)))
		{
			frameFromFile = imread(argv[++i], CV_LOAD_IMAGE_ANYCOLOR);
			from_file = true;
		}
	}
	/* load classifier */
	vision->loadClassifier();

	if(from_file)
	{
		imageAction(frameFromFile);
	}

	/* let's start it all */
	ros::spin();

	delete vision;
}
