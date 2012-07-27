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
				north.update(n);
				break;
			case 'S':
				south.update(n);
				break;
			case 'E':
				east.update(n);
				break;
			case 'W':
				west.update(n);
				break;
			}
		}
	}
	publishLast();
}

void SonarProcesser::publishLast()
{
	Echoes::Sonar msg;
	msg.north = (int) north.curValue();
	msg.south = (int) south.curValue();
	msg.east = (int) east.curValue();
	msg.west = (int) west.curValue();
	sonar_data_pub.publish(msg);
}
