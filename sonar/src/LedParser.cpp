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
#define ALL_OFF 0x00
#define GREEN_ON 0x08
#define GREEN_OFF 0xF7
#define RED_OFF 0xF8
#define FIRST_YELLOW 16

LedParser::LedParser(ReadSonar* read_sonar)
{
	int i;
	this->RedS=0;
	this->GreenS = false;
	for(i=0;i<4; i++) this->YellowS[i] = false;
	this->C=ALL_OFF;
	this->Sender = read_sonar;
}

void LedParser::Green(bool g)
{
	this->GreenS = g;
	if(GreenS==true) this->C = this->C|GREEN_ON;
	else this->C=this->C&GREEN_OFF;
}

void LedParser::Red(char r)
{

	if (r <= NUMREDLED && r >= 1)
		this->RedS = r;
	else if (r > NUMREDLED)
		this->RedS = NUMREDLED;
	else this->RedS = 0;

	this->C = (this->C & 0xF8) | RedS;
		this->RedS=r;
		this->C = (this->C & RED_OFF) | RedS;

}

void LedParser::Yellow(bool y[NUMREDLED])
{
	int i;
	char l=FIRST_YELLOW;
	for(i=0; i<NUMREDLED; i++) 
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
