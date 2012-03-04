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

#include "Zone.h"

using namespace std;
using namespace cv;


void SelectArea(int event, int x, int y, int, void* p)
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
	else
	{
		regione->pointRGB(x,y);
	}
	return;
}

void getColor(vector<Vec3b>& V, Mat& img, Zone* Z)
{
	int x,y;
	for(x=Z->Start.x; x<Z->End.x; x++)
		for(y=Z->Start.y; y<Z->End.y; y++)
		{
			V.push_back(img.at<Vec3b>(x,y));
		}
}

void eliminateDuplicates(vector<Vec3b>& V)
{
	vector<Vec3b>::iterator i1,i2;
	for(i1=V.begin(); i1<V.end(); i1++)
		for(i2=(i1+1); i2<V.end(); i2++)
		{
			if(((*i2)[0]==(*i1)[0]) && ((*i2)[1]==(*i1)[1]) && ((*i2)[2]==(*i1)[2]) ) 
			{
				i2=V.erase(i2);
				i2--;
			}
		}
}

void printOnfile(vector<Vec3b>& V, char c, ofstream& output)
{
	vector<Vec3b>::iterator it;
	for(it=V.begin(); it<V.end(); it++)
	{
		output << " " << (int)((*it)[0]) << " " << (int)((*it)[1]) << " " << (int)((*it)[2]) << " " << c; 
	}

}

int main(int argc,char** argv)
{
	char c;
	bool Loop=true;
	Mat img;
	Zone* regione;
	vector<Vec3b> r;
	vector<Vec3b> b;
	vector<Vec3b> g;
	if(argc<2)
	{
		cout << "Utilizzo:\nlittleendian [file name] [options]\n";
		exit(EXIT_FAILURE);
	}
	img=imread(argv[1],CV_LOAD_IMAGE_COLOR);
	if(img.empty())
	{
		cerr << "Errore: File non valido o non esistente\n";
		exit(EXIT_FAILURE);
	}
	namedWindow("Little Endian Interface", CV_WINDOW_AUTOSIZE);
	regione= new Zone("Little Endian Interface",img);
	cout << "Hot keys: \n"
			"\tESC - esce dal programma\n"
			"\tr - attribuisce ai pixel l'etichetta r\n"
			"\tg - attribuisce ai pixel l'etichetta g\n"
			"\tb - attribuisce ai pixel l'etichetta b\n"
			"\tc - crea il file .dts\n"
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
			case 'r':
				cout << "seleziono rosso\n";
				getColor(r, img, regione);
				eliminateDuplicates(r);
				break;
			case 'g':
				cout << "seleziono giallo\n";
				getColor(g, img, regione);
				eliminateDuplicates(g);
				break;
			case 'b':
				cout << "seleziono blu\n";
				getColor(b, img, regione);
				eliminateDuplicates(b);
				break;
			case 'c':
				cout << "creo file .dts\n";
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
	ofstream output;
	output.open ("DataSet.dts", ios::out);
	if (output.is_open())
	{
		output << (r.size()+g.size()+b.size());
		printOnfile(r,'R', output);
		printOnfile(g,'G', output);
		printOnfile(b,'B', output);
		output.close();
		cout << "\nFile Creato!\n";
	}
	else
	{
		cerr << "\nErrore! file non aperto in scrittura!\n";
	}
}
