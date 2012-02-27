/*
 * RoboTower, Hi-CoRG based on ROS
 *
 *
 * Copyright (C) 2011 Marcello Pogliani, Davide Tateo
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
#include "LedParser.h"

LedParser::LedParser()
{
	this.RedS=0;
	this.GreenS=FALSE;
	this.YellowS[4]=FALSE;
	this.C=0x00;
}

LedParser::Green(bool g)
{
	this.GreenS=g;
	if(GreenS=TRUE) this.C=this.C|0x08;
	else this.C=this.C&0xF7;
}

void LedParser::Red(char r)
{
	this.RedS=r;
	this.C=this.C&(8+Red);
}

void LedParser::Yellow(bool y[4])
{
	int i;
	char l=32;
	for(i=0; i<4; i++) 
	{
		this.YellowS[i]=y[i];
		if(this.YellowS[i]=TRUE) this.C=this.C|l;
		else this.C=this.C&(!l);
		l*2;
	}
}

void LedParser::SendToLed() 
{
	this.Sender->sendStringCommand(&C, 1);
}
