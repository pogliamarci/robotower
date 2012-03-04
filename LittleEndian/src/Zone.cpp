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
}

void Zone::setStart(int x, int y)
{
	this->Start=Point(x,y);
}

void Zone::setEnd(int x, int y)
{
	Point tmp;
	tmp.x=x;
	tmp.y=y;
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
	rectangle(img,this->Start,this->End,CV_RGB(0,255,0), 1,8,0);
	imshow(W,img);
	waitKey(0);
}

void Zone::pointRGB(int x, int y)
{
	Vec3b V;
	V=this->I.at<Vec3b>(x,y);
	std::cout << "(" << (int)V[0] << "," << (int)V[1] << "," << (int)V[2] << ")" << std::endl;
}

