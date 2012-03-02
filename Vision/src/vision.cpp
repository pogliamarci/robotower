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

#include "ColorClassifier.h"
#include "ColorDataset.h"
#include "Blob.h"
#include "PixelMap.h"

#define MIN_BLOB_SIZE 100
#define BLOB_LIST_SIZE	15

using namespace std;
using namespace cv;

// structs
struct blob_info
{
	int numPixel;
	char blobClass;
	Point a;
	Point b;
	Point center;
};

struct blobQueue
{
	struct blob_info list[BLOB_LIST_SIZE];
	int index;
};

//Message to brian
bool energyPointFounded;
bool posLeft;
bool posRight;

unsigned char* temp;
char * value;
struct blob_info YBLOB, RBLOB, UBLOB;
struct blobQueue blobList;
int i;

//Map for BlobAnalysis
map < char, map< int,Blob* > >:: iterator BlobsItr1;
map< int,Blob* > :: iterator BlobsItr2;

KnnColorClassifier* cc;
PixelMap* pm;
ColorDataset* cd;

//Filter Blob
bool filter;
bool createClassifier;

void analyzeCurrentImage(Mat& img, bool filter)
{
	//Point2D
	Point pt1, pt2, pt3, pt4;

	//Inizializzazione blob identificate


	float rap, blobWidth, blobHeight;
	bool shape = false;
	
	//Inizializzazione delle strutture dati per i blob di interesse

	YBLOB.numPixel=0;
	YBLOB.a.x=-1;
	YBLOB.a.y=-1;
	YBLOB.b.x=-1;
	YBLOB.b.y=-1;
	YBLOB.center.x=-1;
	YBLOB.center.y=-1;

	RBLOB.numPixel=0;
	RBLOB.a.x=-1;
	RBLOB.a.y=-1;
	RBLOB.b.x=-1;
	RBLOB.b.y=-1;
	RBLOB.center.x=-1;
	RBLOB.center.y=-1;

	//Linea in mezzo al filmato
	pt3.x = img.cols/2;
	pt3.y = 0;
	pt4.x = img.cols/2;
	pt4.y = img.rows;
	
	line(img, pt3, pt4, CV_RGB(0,0,254), 2, 8, 0);

	//Blob parameters
	pm->SetImage((unsigned char*)img.data, img.cols, img.rows, cc,3);
	
	/*
	Algoritmo di BlobGrowing (2°-3°-4°-5° identificano la regione dell'immagine
	originale in cui eseguire la ricerca dei blob (-1...-1  = tutta l'immagine)
	*/
	pm->BlobGrowing(1,-1,-1,-1,-1);
	for(BlobsItr1=pm->GetBlobs()->begin();BlobsItr1!=pm->GetBlobs()->end();BlobsItr1++)
   		for(BlobsItr2 = BlobsItr1->second.begin();BlobsItr2!=BlobsItr1->second.end();BlobsItr2++)
   			if(BlobsItr2->second->GetValid() && BlobsItr2->second->GetNumPix()>MIN_BLOB_SIZE)
				{
				shape = false;

				//Load blob coord
				pt1.x = BlobsItr2->second->GetMinX();
				pt1.y = BlobsItr2->second->GetMaxY();
				pt2.x = BlobsItr2->second->GetMaxX();
				pt2.y = BlobsItr2->second->GetMinY();
				
				if(filter == true)
				{
					//Blob shape
					blobWidth = (float)(BlobsItr2->second->GetMaxX())-(BlobsItr2->second->GetMinX());
					blobHeight = (float)(BlobsItr2->second->GetMaxY())-(BlobsItr2->second->GetMinY());	
					
					//Shape controll -> must be rectangular with height>width
					rap = blobWidth/blobHeight;
					if((rap < 0.8 && rap > 0.2))
					{
						shape = true;
					}
					
				
					if (shape == true)
					{
						//Use different color for different class and save the bigger one
						if(BlobsItr1->first == 'R')
						{
							//Confronta il blob con il precedente e salvalo
							if(RBLOB.numPixel == 0 || RBLOB.numPixel <  BlobsItr2->second->GetNumPix())
							{
								RBLOB.numPixel = BlobsItr2->second->GetNumPix();
								RBLOB.blobClass = 'R';
								RBLOB.a = pt1;
								RBLOB.b = pt2;
								RBLOB.center.x = (pt2.x-pt1.x)/2 + pt1.x;
								RBLOB.center.y = (pt2.y-pt1.y)/2 + pt1.y;
							}
						}
						else if(BlobsItr1->first == 'Y')
						{
							//Confronta il blob con il precedente e salvalo
							if(YBLOB.numPixel == 0 || YBLOB.numPixel <  BlobsItr2->second->GetNumPix())
							{
								YBLOB.numPixel = BlobsItr2->second->GetNumPix();
								YBLOB.blobClass = 'Y';
								YBLOB.a = pt1;
								YBLOB.b = pt2;
								YBLOB.center.x = (pt2.x-pt1.x)/2 + pt1.x; 
								YBLOB.center.y = (pt2.y-pt1.y)/2 + pt1.y;
							}
						}
					}
				}
				
				else {rectangle(img, pt1, pt2, CV_RGB(0,255,0), 2, 8, 0 );}
			}
			
			//Aggiungilo nella coda
			if(blobList.index == 15)
			{
					blobList.index = 0;
			}
			
			if(YBLOB.numPixel>0)
			{		
				blobList.list[blobList.index] = YBLOB;
			}
			else if (RBLOB.numPixel>0)
			{		
				blobList.list[blobList.index] = RBLOB;
			}
			else
			{
				blobList.list[blobList.index] = UBLOB;
			}
			
			blobList.index ++;
			i = blobList.index;
			i --;

			//Disegna i blob rilevati sull'immagine'
			if (filter)
			{
				//Se nel frame corrente il blob è stato trovato....
				if(blobList.list[i].numPixel > 0)
				{
					if (blobList.list[i].blobClass == 'Y')
					{
						rectangle(img, blobList.list[i].a, blobList.list[i].b, CV_RGB(254,254,0), 2, 8, 0 );
					}
					
					else if (blobList.list[i].blobClass == 'R')
					{
						rectangle(img, blobList.list[i].a, blobList.list[i].b, CV_RGB(254,0,0), 2, 8, 0 );
					}
				}
				
				//Altrimenti cerca nella coda...
				else
				{
					//Prendo il blob del frame precedente
					for(int xx = 0; xx<5; xx++)
					{
						//Se necessario "riavvolgo" la coda
						if (i == -1)
						{
							i = 15;
						}
						if (blobList.list[i].blobClass == 'Y')
						{
								rectangle(img, blobList.list[i].a, blobList.list[i].b, CV_RGB(254,254,0), 2, 8, 0 );
								break;
						}
						else if (blobList.list[i].blobClass == 'R')
						{
								rectangle(img, blobList.list[i].a, blobList.list[i].b, CV_RGB(254,0,0), 2, 8, 0 );
								break;
						}
						
						i--;
					}
				}
			}
}


// callback again, trying version without saving on file
void imageMessageCallback(const sensor_msgs::CompressedImage::ConstPtr& message)
{

	Mat frame = imdecode(message->data, CV_LOAD_IMAGE_ANYCOLOR);

	if (!frame.empty())
	{
		analyzeCurrentImage(frame, filter);
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
	ros::Subscriber source = ros_node.subscribe("spykee_camera", 1,imageMessageCallback);
	namedWindow("SpyKeeView", CV_WINDOW_AUTOSIZE);
	
	//Variabili di controllo
	createClassifier = false;
	filter = true;
	
	blobList.index = 0;
	UBLOB.blobClass = 'U';
	UBLOB.numPixel=0;
	UBLOB.a.x=-1;
	UBLOB.a.y=-1;
	UBLOB.b.x=-1;
	UBLOB.b.y=-1;
	UBLOB.center.x=-1;
	UBLOB.center.y=-1;
	
	for (int k = 0; k<15; k++)
	{
		blobList.list[k] = UBLOB;
	}
	
	cd = new ColorDataset();
	cc = new KnnColorClassifier();
	pm = new PixelMap();
	
	value = (char*)malloc(sizeof(char)*100);

	//Carica classificatore
	cc->load_matrix("classifier[5-17].kcc");
	cout << "----> Classifier loaded  [OK]"<<endl;
	/* let's start it all */
	ros::spin();
}
