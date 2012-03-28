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

#include "isaac.h"
#include <iostream>
#include <cmath>

SonarBuffer::SonarBuffer()
{
	int i;
	for(i=0; i<MAXCAMPIONI; i++)
	{
		this->data[i]=0;
	}
	this->index=0;
}

void SonarBuffer::setTempoBloccato()
{
	float varianza, media, devStandard;
	varianza=this->calcolaVarianza();
	devStandard=sqrt(varianza);
	media=this->calcolaMedia();
	std::cerr << "VARIANZA: " << varianza  << std::endl;
	std::cerr << "MEDIA: " << media  << std::endl;
	std::cerr << "DEVIAZIONE STANDARD: " <<  devStandard << std::endl;
	if(varianza>media/5)
	{
		this->tempo=0;
	}
	else
	{
		this->tempo++;
	}
}

int SonarBuffer::getTempoBloccato()
{
	return this->tempo;
}

void SonarBuffer::insert(int element)
{
	this->data[this->index]=element;
	this->index++;
	if(this->index == MAXCAMPIONI) this->index=0;
}

float SonarBuffer::calcolaVarianza()
{
	float media=0, mediaquadra=0;
	for(int i=0; i<MAXCAMPIONI; i++)
	{
		media+=this->data[i];
		mediaquadra+=this->data[i]*this->data[i];
	}
	media/=MAXCAMPIONI;
	mediaquadra/=MAXCAMPIONI;
	return (mediaquadra-media*media);
}

float SonarBuffer::calcolaMedia()
{
	float media=0;
	for(int i=0; i<MAXCAMPIONI; i++)
	{
		media+=this->data[i];
	}
	media/=MAXCAMPIONI;
	return media;
}


