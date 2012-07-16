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

#include "ImageAnalyzer.h"

#define EXIT_FROM_WINDOW -1

using namespace std;
using namespace cv;

/* variabili globali in cerca di sistemazione... */
ros::Publisher results_publisher;
ImageAnalyzer* analyzer;

VideoWriter* record;

void sendMessage(Vision::Results message)
{
	results_publisher.publish(message);
}

void imageAction(Mat& frame)
{
	if (!frame.empty())
	{
		sendMessage(analyzer->analyze(frame));
		imshow("SpyKeeView", frame);
		char c = waitKey(5);
		/*// This appears not to work anymore with the latest openCV, boh =)
		if((c == 'c') || (c == EXIT_FROM_WINDOW));
			exit(EXIT_SUCCESS);
		*/
	}
}

void imageMessageCallback(const sensor_msgs::CompressedImage::ConstPtr& message)
{
	cv::Mat f = imdecode(message->data, CV_LOAD_IMAGE_ANYCOLOR);
	*record << f;
	imageAction(f);
}

int main (int argc, char** argv)
{
	/* variables to parse CLI arguments */
	bool classifier_loaded = false;
	bool from_file = false;
	
	/* file paths */
	char DefaultDataset[] = "DataSet.dts";
	char DefaultClassifier[] = "classifier[5-17].kcc";
	
	/* vision algorithm variables */
	KnnColorClassifier* cc = new KnnColorClassifier();
	PixelMap* pm = new PixelMap();
	ColorDataset* cd = new ColorDataset();

	/* initialize image analyzer */
	analyzer = new ImageAnalyzer(cc, pm, cd);

	/* ROS variables */
	ros::init(argc, argv, "vision");
	ros::NodeHandle ros_node;
	ros::Subscriber source = ros_node.subscribe("spykee_camera", 1, imageMessageCallback);
	results_publisher = ros_node.advertise<Vision::Results>("vision_results", 10);

	/* opencv variables */
	Mat frame;
	
	namedWindow("SpyKeeView", CV_WINDOW_AUTOSIZE);
	
	for(int i=1; i<argc; i++)
	{
		if(strcmp(argv[i],"-h") == 0)
		{
			cout 	<< "Utilizzo:\n"
				<< "-h\t\t visualizza questo messaggio\n"
				<< "-f [filename]\t carica il frame da analizzare da file\n"
				<< "-l\t\t Carica il file DataSet.dts e genera il file .kcc\n"
				<< "-L [filename]\t carica il Color Data Set da file e genera il file .kcc\n"
				<< "-k [filename]\t carica il file .kcc da file\n"
				<< endl;
			exit(EXIT_SUCCESS);
		}
		else if((strcmp(argv[i],"-f") == 0) && (argc > (i+1)))
		{
			frame = imread(argv[++i], CV_LOAD_IMAGE_ANYCOLOR);
			from_file = true;
		}
		else if(strcmp(argv[i],"-l") == 0)
		{
			cd->load(DefaultDataset);
			cout << "----> Color Data set  [OK]" << endl;
			//Creazione del classificatore colore
			cc->fast_build(cd, 5, 17, false);
			cc->save_matrix(DefaultClassifier);
		}
		else if((strcmp(argv[i],"-L" ) == 0) && (argc > (i+1)))
		{
			cd->load(argv[++i]);
			cout << "----> Color Data set  [OK]" << "caricato file:" << argv[i] << endl;
			//Creazione del classificatore colore
			cc->fast_build(cd, 5, 17, false);
			cc->save_matrix(DefaultClassifier);
		}
		else if((strcmp(argv[i], "-k") == 0) && (argc > (i+1)))
		{
			/* load classifier */
			cc->load_matrix(argv[++i]);
			cout << "----> Classifier loaded  [OK]" << endl;
			classifier_loaded = true;
		}
	}
	
	if (!classifier_loaded)
	{
		/* load classifier */
		cout << "----> No classifier loaded, loading default classifier" << endl;
		cc->load_matrix("classifier[5-17].kcc");
		cout << "----> Classifier loaded  [OK]"<<endl;
	}
	
	if(from_file)
	{
		imageAction(frame);
	}

	// WARNING -- Temporary FIXME
	Size frameSize(320,240);
	record = new VideoWriter("RobotVideo.avi", CV_FOURCC('D','I','V','X'), 20, frameSize, true);
	if(!record->isOpened()) {
		cerr << "Error opening videofile" << endl;
	}

	/* let's start it all */
	ros::spin();

	// This code analyzes images from a pre-recorded video
	/*
	VideoCapture capture("RobotVideoPlay.avi");
	double rate = capture.get(CV_CAP_PROP_FPS);
	int delay = 1000/rate;
	while(capture.grab()) {
		Mat img;
		capture.retrieve(img);
		imageAction(img);
		if(waitKey(delay)>=0)
			break;
	}
	*/

	/* NEEDED TO FINALIZE THE FILE */
	delete record;
}
