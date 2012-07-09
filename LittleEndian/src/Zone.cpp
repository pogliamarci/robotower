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
#include "Zone.h"

using namespace cv;

Zone::Zone(const char* Window, Mat& img)
{
	this->W=Window;
	this->I=img;
	this->Drag_Drop=false;
}

void Zone::setStart(int x, int y)
{
	this->Drag_Drop=true;
	this->Start=Point(x,y);
}

void Zone::setEnd(int x, int y)
{
	Point tmp;
	tmp.x=x;
	tmp.y=y;
	this->Drag_Drop=false;
	if(this->Start.x>tmp.x)
	{
		this->End.x=this->Start.x;
		this->Start.x=tmp.x;
	}
	else
		this->End.x=tmp.x;
	if(this->Start.y>tmp.y)
	{
		this->End.y=this->Start.y;
		this->Start.y=tmp.y;
	}
	else
		this->End.y=tmp.y;
}

void Zone::drawZone()
{
	Mat img=this->I.clone();
	rectangle(img,this->Start,this->End,BLACK_R, 1,8,0);
	imshow(W,img);
	waitKey(0);
}

void Zone::drawZone(int x, int y)
{
	Point Now;
	if(this->Drag_Drop)
	{
		Now.x=x;
		Now.y=y;
		Mat img=this->I.clone();
		rectangle(img,this->Start,Now,WHITE_R, 1,8,0);
		imshow(W,img);
		waitKey(0);
	}
}

void Zone::printZone(Scalar C)
{
	rectangle(this->I,this->Start,this->End,C, 1,8,0);
}

void Zone::pointRGB(int col, int row)
{
	Vec3b& element = I.at<Vec3b>(row, col);
	int b_pix = element[0];
	int g_pix = element[1];
	int r_pix = element[2];
	//cancella la precedente stampa di coordinate
	std::cout << "\r" << "                    " << "\t" << "   " << "                  ";
	//stampa le nuove coordinate
	std::cout << "\rCoordinate:" <<"(" << col << "," << row << ")" << "\tRGB:" <<"(" << r_pix << ", " << g_pix << ", " << b_pix << ")" << std::flush;
}

void Zone::updateImg(Mat& img)
{
	this->I=img;
}

