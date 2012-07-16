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

#include "SonarProcesser.h"
#include <iostream>
#include <cstdlib>

SonarProcesser::SonarProcesser(ros::Publisher pub)
{
	north = 0;
	south = 0;
	east = 0;
	west = 0;
	sonar_data_pub = pub;
}

// FIXME manca la gestione degli errori
void SonarProcesser::process(string str)
{
	vector<string> tokens;
	tokenize(str, tokens, ",");
	for(unsigned int i=0; i < tokens.size(); i++)
	{
		const char* t = tokens.at(i).c_str();
		if(t[0] != '\0' && t[1] != '\0')
		{
			int n = atoi(tokens.at(i).c_str() + 2);
			switch(t[0])
			{
			case 'N':
				north = n;
				break;
			case 'S':
				south = n;
				break;
			case 'E':
				east = n;
				break;
			case 'W':
				west = n;
				break;
			}
		}
	}
	publishLast();
}

void SonarProcesser::publishLast()
{
	Echoes::Sonar msg;
	msg.north = north;
	msg.south = south;
	msg.east = east;
	msg.west = west;
	sonar_data_pub.publish(msg);
}

SonarProcesser::~SonarProcesser()
{
	// TODO Auto-generated destructor stub
}
