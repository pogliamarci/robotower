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

#include "TowerProcesser.h"
#include <iostream>
#include <cstdlib>
#include "Echoes/Towers.h"

TowerProcesser::TowerProcesser(ros::Publisher pub)
{
	factories = 0;
	towers = 0;
	publisher = pub;
}

// FIXME manca la gestione degli errori
void TowerProcesser::process(string str)
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
			case 'F':
				factories = n;
				break;
			case 'T':
				towers = n;
				break;
			}
		}
	}

	/* publish the message to ROS */
	Echoes::Towers msg;
	msg.towers = towers;
	msg.factories = factories;
	publisher.publish(msg);
}

TowerProcesser::~TowerProcesser()
{
	// TODO Auto-generated destructor stub
}
