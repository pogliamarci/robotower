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

#include "vision.h"

#define EXIT_FROM_WINDOW -1

using namespace std;
using namespace cv;

KnnColorClassifier* cc;
PixelMap* pm;
ColorDataset* cd;

bool checkBlobShape(int width, int heigth) {
	//Shape control -> must be rectangular with height > width
	float ratio = (float) width / heigth;
	if (ratio > MIN_BLOB_RATIO && ratio < MAX_BLOB_RATIO)
	{
		return true;
	}
	return false;
}

void analyzeCurrentImage(Mat& img)
{
	static BlobBuffer buffer;

	/* Point 2D */
	Point pt1, pt2, pt3, pt4;
	
	/* Maps needed for BlobAnalysis */
	map < char, map< int,Blob* > >:: iterator BlobsItr1;
	map< int,Blob* > :: iterator BlobsItr2;
	
	/* initialise classes to store data regarding the found blobs */
	BlobInfo red_blob('R');
	BlobInfo yellow_blob('Y');

	/* draw a line in the middle of the frame (why?) */
	pt3.x = img.cols/2;
	pt3.y = 0;
	pt4.x = img.cols/2;
	pt4.y = img.rows;
	line(img, pt3, pt4, CV_RGB(0,0,254), 2, 8, 0);

	/* Blob parameters, PixMap object initialisation */
	pm->SetImage((unsigned char*)img.data, img.cols, img.rows, cc, 3);
	
	/* Start blob growing algorithm: parameters from the 2nd to the 5th
	 * identify the region of the image in which blobs are to be searched for
	 * (2nd to 5th params are set to -1 ==> search blobs over the whole image)
	 */
	pm->BlobGrowing(1,-1,-1,-1,-1);
	for(BlobsItr1=pm->GetBlobs()->begin();BlobsItr1!=pm->GetBlobs()->end();BlobsItr1++)
	{
   		for(BlobsItr2 = BlobsItr1->second.begin();BlobsItr2!=BlobsItr1->second.end();BlobsItr2++)
   		{
   			if(BlobsItr2->second->GetValid() && BlobsItr2->second->GetNumPix()>MIN_BLOB_SIZE)
			{
				/* load blob coordinates */
   				/* top left point */
				pt1.x = BlobsItr2->second->GetMinX();
				pt1.y = BlobsItr2->second->GetMaxY();
				/* bottom right point */
				pt2.x = BlobsItr2->second->GetMaxX();
				pt2.y = BlobsItr2->second->GetMinY();
				cout << "Blob found: " << BlobsItr1->first <<endl;
				
				/* compute width and height to perform some check... */
				int blob_width = pt2.x - pt1.x;
				int blob_heigth = pt1.y - pt2.x;

				/* is the blob shape similar to the expected one? */
				if (true/*checkBlobShape(blob_width, blob_heigth)*/)
				{
					/* save the biggest found blob for each colour class */
					if(BlobsItr1->first == 'R' && ( red_blob.getNumPix() == 0
							|| red_blob.getNumPix() <  BlobsItr2->second->GetNumPix()))
					{
						red_blob.save(BlobsItr2->second->GetNumPix(), pt1, pt2);
					}
					else if(BlobsItr1->first == 'Y' && (yellow_blob.getNumPix() == 0
							|| yellow_blob.getNumPix() <  BlobsItr2->second->GetNumPix()))
					{
						yellow_blob.save(BlobsItr2->second->GetNumPix(), pt1, pt2);
					}
				}
			}
   		}
	}

	/* Add the found blob to the queue */
	/* Assumption: in each frame there is only one blob of interest */
	if (yellow_blob.getNumPix() > 0)
	{
		buffer.insert(yellow_blob);
	}
	else if (red_blob.getNumPix() > 0)
	{
		buffer.insert(red_blob);
	}
	else
	{
		BlobInfo undefined_blob('U');
		buffer.insert(undefined_blob);
	}

	BlobInfo* result = buffer.lastValidBlob();
	if (result != NULL)
	{
		switch (result->getClass())
		{
		case 'Y':
			rectangle(img, result->a, result->b, CV_RGB(254,254,0), 2, 8, 0);
			break;
		case 'R':
			rectangle(img, result->a, result->b, CV_RGB(254,0,0), 2, 8, 0);
			break;
		default:
			break;
		}
	}

	/*
	 * Here the original code sent a message via the middleware
	 * passing result->centre plus two booleans to indicate
	 * whether the algorithm recognised the "home" (red blob)
	 * or the "energy point" (yellow blob)
	 */
}

void imageMessageCallback(const sensor_msgs::CompressedImage::ConstPtr& message)
{
	Mat frame = imdecode(message->data, CV_LOAD_IMAGE_ANYCOLOR);
	
	if (!frame.empty())
	{
		analyzeCurrentImage(frame);
		imshow("SpyKeeView", frame);
		char c = waitKey(5);
		if (c == 'c')
			exit(EXIT_SUCCESS);
	}
}

int main (int argc, char** argv)
{
	//variables for parsing line command
	int i;
	bool classifier_loaded = false;
	bool from_file = false;
	
	//ros variable
	ros::init(argc, argv, "vision");
	ros::NodeHandle ros_node;
	ros::Subscriber source = ros_node.subscribe("spykee_camera", 1, imageMessageCallback);
	
	//opencv variables
	Mat frame;
	
	namedWindow("SpyKeeView", CV_WINDOW_AUTOSIZE);
	
	cd = new ColorDataset();	/* is it really needed? */
	cc = new KnnColorClassifier();
	pm = new PixelMap();
	
	for(i=1; i<argc; i++)
	{
		if(strcmp(argv[i],"-h") == 0)
		{
			cout 	<< "Utilizzo:\n"
				<< "-h\t visualizza questo messaggio\n"
				<< "-f [filename]\t carica il frame da analizzare da file\n"
				<< "-l\t Carica il file DataSet.dts e genera il file .kcc\n"
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
			cd->load("DataSet.dts");
			cout << "----> Color Data set  [OK]" << endl;
			//Creazione del classificatore colore
			cc->fast_build(cd, 5, 17, false);
			cc->save_matrix("classifier[5-17].kcc");
		}
		else if((strcmp(argv[i],"-L" ) == 0) && (argc > (i+1)))
		{
			cd->load(argv[++i]);
			cout << "----> Color Data set  [OK]" << "caricato file:" << argv[i] << endl;
			//Creazione del classificatore colore
			cc->fast_build(cd, 5, 17, false);
			cc->save_matrix("classifier[5-17].kcc");
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
		analyzeCurrentImage(frame);
		imshow("SpyKeeView", frame);
		char c = waitKey(0);
		if((c == 'c') || (c == EXIT_FROM_WINDOW)) exit(EXIT_SUCCESS);
	}

	/* let's start it all */
	ros::spin();
}
