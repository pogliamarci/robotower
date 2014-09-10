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
#include "opencv2/opencv.hpp"
#include <fstream>
#include "ros/ros.h"

#include "LittleObject.h"

#define ESC 27
#define CLOSE_FROM_WINDOW -1

using namespace std;
using namespace cv;

void SelectArea(int event, int x, int y, int, void* p)
{
	Zone* regione;
	regione=(Zone*)p;
	if(event == CV_EVENT_LBUTTONDOWN)
	{
		regione->setStart(x,y);
	}
	else if(event == CV_EVENT_LBUTTONUP)
	{
		regione->setEnd(x,y);
		regione->drawZone();
	}
	else
	{
		regione->pointRGB(x,y);
		regione->drawZone(x,y);
	}
	return;
}

int main(int argc,char** argv)
{
	//variabili main
	int i;
	char c;
	bool Loop = true;
	
	//oggetti
	Zone* regione;
	LittleObject* data=NULL;
	ros::NodeHandle* ros_node;
	ros::Subscriber sub;
	
	//parsing degli argomenti in ingresso
	for(i=0; i<argc; i++)
	{
		if((strcmp(argv[i],"-f") == 0) && (argc>i+1))
		{
			data=new LittleObject(false);
			data->getImg(argv[++i]);
		}
	}

	if(data == NULL)
	{
		/* ROS initialization */
		data = new LittleObject(true);
		ros::init(argc, argv, "little_endian");
		ros_node = new ros::NodeHandle();
		sub = ros_node->subscribe("spykee_camera", 1, &LittleObject::getImgRos, data);
	}
	namedWindow("Little Endian Interface", CV_WINDOW_AUTOSIZE);
	regione= new Zone("Little Endian Interface",data->img);
	data->Z=regione;
	cout << "Hot keys: \n"
			"\tESC - esce dal programma\n"
			"\tr - attribuisce ai pixel l'etichetta r\n"
			"\tg - attribuisce ai pixel l'etichetta g\n"
			"\tb - attribuisce ai pixel l'etichetta b\n"
			"\tc - crea il file .dts\n"
			"\tz - annulla tutte le selezioni\n"
			<< endl;
	cvSetMouseCallback("Little Endian Interface",SelectArea,regione);
	
	//main loop
	while(Loop)
	{
		c=data->showImage();
		switch(c)
		{
			case CLOSE_FROM_WINDOW:
			case ESC:
				cout << "\n";
				exit(0);
				break;
			case 'r':
				cout << "\nseleziono rosso\n";
				data->getColor('r');
				data->eliminateDuplicates('r');
				regione->printZone(RED_R);
				break;
			case 'g':
				cout << "\nseleziono verde\n";
				data->getColor('g');
				data->eliminateDuplicates('g');
				regione->printZone(GREEN_R);
				break;
			case 'b':
				cout << "\nseleziono blu\n";
				data->getColor('b');
				data->eliminateDuplicates('b');
				regione->printZone(BLUE_R);
				break;
			case 'c':
				cout << "\ncreo file .dts\n";
				Loop=false;
				break;
			case 'z':
				cout << "\nannullo tutte le selezioni\n";
				data->updateImg();
				regione->updateImg(data->img);
				imshow("Little Endian Interface",data->img);
				data->cleanVectors();
				break;
			case 'e':
				cout << "\nLittleEndian: il lato giusto dell'uovo!\n\nps: W lilliput!\n";
				break;
			case 0:
				break;
			default:
				cout << "\ncomando sconosciuto\n";
				break;
		}
	}
	
	//creazione file di output DataSet.dts
	ofstream output;
	output.open ("DataSet.dts", ios::out);
	if (output.is_open())
	{
		data->printOnfileNumber(output);
		data->printOnfile('r','R', output);
		data->printOnfile('g','G', output);
		data->printOnfile('b','B', output);
		output.close();
		cout << "\nFile Creato!\n";
	}
	else
	{
		cerr << "\nErrore! file non aperto in scrittura!\n";
	}

}
