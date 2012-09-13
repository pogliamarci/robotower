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

#include "BlockedTimeCalculator.h"
#include <iostream>

BlockedTimeCalculator::BlockedTimeCalculator()
{
	tempo = 0;
	int i;
	for (i = 0; i < MAXCAMPIONI; i++)
	{
		data[i] = 0;
	}
	index = 0;
}

void BlockedTimeCalculator::setTempoBloccato()
{
	if (this->calcolaVarianza() > THRESHOLD)
	{
		tempo = 0;
	}
	else
	{
		tempo++;
	}
}

int BlockedTimeCalculator::getTempoBloccato()
{
	return tempo;
}

void BlockedTimeCalculator::insert(int element)
{
	data[index] = element;
	index++;
	if (index == MAXCAMPIONI)
		index = 0;
	setTempoBloccato();
}

float BlockedTimeCalculator::calcolaVarianza()
{
	float media = 0, mediaquadra = 0;
	for (int i = 0; i < MAXCAMPIONI; i++)
	{
		media += data[i];
		mediaquadra += data[i] * data[i];
	}
	media /= MAXCAMPIONI;
	mediaquadra /= MAXCAMPIONI;
	return (mediaquadra - media * media);
}

float BlockedTimeCalculator::calcolaMedia()
{
	float media = 0;
	for (int i = 0; i < MAXCAMPIONI; i++)
	{
		media += data[i];
	}
	media /= MAXCAMPIONI;
	return media;
}

