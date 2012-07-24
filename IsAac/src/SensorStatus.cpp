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

#include "SensorStatus.h"

#include <fstream>

SensorStatus::SensorStatus()
{
	for (int i=0; i < CARDINAL_POINTS; i++)
		sonar[i] = 0;
	tower_found = false;
	tower_position = 0;
	factory_found = false;
	factory_position = 0;
	lastAction = nothing;
}

SensorStatus::SensorStatus(std::string configFile) {
	SensorStatus();

	std::fstream config;
	config.open(configFile.c_str(), std::ios::in);
	std::cerr << "Trying to open file: " << configFile << std::endl;
	if(!config.is_open()) {
		std::cerr << "Error opening the configuration file, "
				"no action will be assigned to RFID tags!" << std::endl;
		return;
	}
	while(config.good()) {
		std::string st;
		getline(config, st);
		populateMapWithLine(st);
	}
	config.close();
}

void SensorStatus::populateMapWithLine(std::string configLine) {
    size_t idindex = configLine.find("id:") + 4;
    size_t actionStartIndex = configLine.find("action:");
    size_t actionindex = actionStartIndex + 8;
    if(configLine.size() >= actionindex) {
      std::string id = configLine.substr(idindex, actionStartIndex - idindex);
      id.erase(id.find_last_not_of(" \n\r\t") + 1);	 // trim trailing whitespace

      std::string action = configLine.substr(actionindex);
      action.erase(action.find_last_not_of(" \n\r\t") + 1);

      idToAction.insert(std::make_pair(id, strToAction(action)));
    }
}

void SensorStatus::fromSonarCallback(const Echoes::Sonar& message)
{
	sonar[NORTH] = message.north;
	sonar[SOUTH] = message.south;
	sonar[EAST] = message.east;
	sonar[WEST] = message.west;
}

void SensorStatus::fromVisionCallback(const Vision::Results& message)
{
	tower_found = message.towerFound;
	tower_position = message.towerPos;
	factory_found = message.factoryFound;
	factory_position = message.factoryPos;
}

void SensorStatus::fromRfidCallback(const Echoes::Rfid& message)
{
	std::cerr << "RFID RICEVUTO: " << message.id << std::endl;;
	if(idToAction.count(message.id) > 0) {
		lastAction = idToAction[message.id];
	} else {
		std::cerr << "Nessuna azione associata" << std::endl;
		lastAction = nothing;
	}
}

RfidAction SensorStatus::strToAction(std::string token) {
	if(token.compare("lock_all") == 0) {
		return lock_all;
	} else if(token.compare("disable_vision") == 0) {
		return disable_vision;
	} else if(token.compare("force_rotate_left") == 0) {
		return force_rotate_left;
	} else if(token.compare("force_rotate_right") == 0) {
		return force_rotate_right;
	}
	std::cout << "Fallback to default action, token "
			<< token << " not recognized" << std::endl;
	return nothing;
}
