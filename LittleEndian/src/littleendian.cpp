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
#include "opencv2/opencv.hpp"

#include "Zone.h"

using namespace std;
using namespace cv;


void SelectArea( int event, int x, int y, int, void* p)
{
	Zone* regione;
	regione=(Zone*)p;
	if(event == CV_EVENT_LBUTTONDOWN)
	{
		regione->setStart(x,y);
		cout << "click del mouse giù\n";
	}
	else if(event == CV_EVENT_LBUTTONUP)
	{
		regione->setEnd(x,y);
		cout << "click del mouse su\n";
		regione->drawZone();
	}
	return;
}

int main(int argc,char** argv)
{
	char c;
	bool Loop=true;
	Mat img;
	Zone* regione;
	if(argc<2)
	{
		cout << "utilizzo:\nlittleendian [file name] [options]\n";
	}
	img=imread(argv[1],CV_LOAD_IMAGE_COLOR);
	namedWindow("Little Endian Interface", CV_WINDOW_AUTOSIZE);
	regione= new Zone("Little Endian Interface",img);
	cout << "Hot keys: \n"
			"\tESC - esce dal programma\n"
			"\tc - crea il file .kcc\n"
			<< endl;
	cvSetMouseCallback("Little Endian Interface",SelectArea,regione);
	while(Loop)
	{
		imshow("Little Endian Interface",img);
		c=waitKey(0);
		switch(c)
		{
			case 27:
				exit(0);
				break;
			case 'c':
				destroyWindow("Little Endian Interface");
				cout << "creo file .kcc\n";
				Loop=false;
				break;
			case 'e':
				cout << "LittleEndian: il lato giusto dell'uovo!\n\nps: W lilliput!\n";
				break;
			default:
				cout << "comando sconosciuto\n";
				break;
		}
	}
}