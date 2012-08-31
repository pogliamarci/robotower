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


#ifndef GAMECONFIGURATION_H_
#define GAMECONFIGURATION_H_

#include<QFile>
#include<QXmlSimpleReader>
#include<QXmlInputSource>

#include<vector>
#include<iostream>
#include<cstdlib>
#include"RTConfigHandler.h"

class GameConfiguration {
private:
	RTConfigGeneral configuration;
	std::vector<RTConfigRfidEntry> rfidList;

public:
	GameConfiguration(std::string filePath);
	RTConfigGeneral getMainConfiguration();
	int getNumActions();
	std::vector<int> getRfidList(int actionId);
};


#endif /* GAMECONFIGURATION_H_ */
