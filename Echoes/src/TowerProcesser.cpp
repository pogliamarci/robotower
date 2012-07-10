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

// FIXME manca la gestione degli errori
void TowerProcesser::process(string str)
{
	vector<string> tokens;
	tokenize(str, tokens, ",");
	for(int i=0; i < tokens.size(); i++)
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
	cout << "Towers: " << towers << " - Factories: " << factories << endl;
}

void TowerProcesser::tokenize(const string& str, vector<string>& tokens, const string& delimiters )
{
	// Skip delimiters at beginning.
	std::string::size_type lastPos = str.find_first_not_of(delimiters, 0);
	// Find first "non-delimiter".
	std::string::size_type pos     = str.find_first_of(delimiters, lastPos);

	while (std::string::npos != pos || std::string::npos != lastPos)
	{
		// Found a token, add it to the vector.
		tokens.push_back(str.substr(lastPos, pos - lastPos));
		// Skip delimiters.  Note the "not_of"
		lastPos = str.find_first_not_of(delimiters, pos);
		// Find next "non-delimiter"
		pos = str.find_first_of(delimiters, lastPos);
	}
}

/*
void SonarProcesser::publishLast()
{
	Echoes::Sonar msg;
	msg.north = (float) north;
	msg.south = (float) south;
	msg.east = (float) east;
	msg.west = (float) west;
	sonar_data_pub.publish(msg);
}
*/

TowerProcesser::~TowerProcesser()
{
	// TODO Auto-generated destructor stub
}
