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
#include <cstdio>
#include "LedParser.h"

LedParser::LedParser(ReadSonar* read_sonar)
{
    int i;
	this->RedS=0;
	this->GreenS = false;
	for(i=0;i<4; i++)
	this->YellowS[i] = false;
	this->C=0x00;
	this->Sender = read_sonar;
}

void LedParser::Green(bool g)
{
	this->GreenS = g;
	if(GreenS==true) this->C = this->C|0x08;
	else this->C=this->C&0xF7;
}

void LedParser::Red(char r)
{
	this->RedS=r;
	this->C = (this->C & 0xF8) | RedS;
}

void LedParser::Yellow(bool y[4])
{
	int i;
	char l=16;
	for(i=0; i<4; i++) 
	{
		this->YellowS[i]=y[i];
		if(this->YellowS[i]==true) this->C=this->C|l;
		else this->C=this->C&(~l);
		l*=2;
	}
}

void LedParser::SendToLed() 
{
	printf("numero %d\n", C);
	(this->Sender)->sendStringCommand(&C, 1);
}
