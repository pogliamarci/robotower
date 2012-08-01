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
#include "GameControl.h"

GameControl::GameControl(int factoryNumber, int towerNumber)
{
	timeToLive = gameMaxTime;
	points = 0;
	isQuitting = false;
	this->factoryNumber = factoryNumber;
	this->towerNumber = towerNumber;
	initializeRfidConfiguration("../../rfidconfig.txt");
}

void GameControl::disableRFID(std::string id)
{
	RfidEntry &entry = rfidMap[id];
	if(entry.status)
	{
		entry.status = false;
		emit updatedRfidStatus(entry.number, entry.status);
	}
}

void GameControl::initializeRfidConfiguration(std::string configFile)
{
	std::fstream config;
	config.open(configFile.c_str(), std::ios::in);
	std::cerr << "Trying to open file: " << configFile << std::endl;
	if (!config.is_open())
	{
		std::cerr << "Error opening the configuration file, "
				"no action will be assigned to RFID tags!" << std::endl;
		return;
	}
	for (int i = 0; config.good(); i++)
	{
		std::string st;
		getline(config, st);
		populateMapWithLine(st, i);
	}
	config.close();
}

void GameControl::populateMapWithLine(std::string configLine, int index)
{
	size_t idindex = configLine.find("id:") + 4;
	size_t actionStartIndex = configLine.find("action:");
	size_t actionindex = actionStartIndex + 8;
	if (configLine.size() >= actionindex)
	{
		std::string id = configLine.substr(idindex, actionStartIndex - idindex);
		id.erase(id.find_last_not_of(" \n\r\t") + 1); // trim trailing whitespace
		RfidEntry entry = {index, true};
		rfidMap.insert(std::make_pair(id, entry));
	}
}

void GameControl::updateGamePoints()
{
	points += towerNumber*towerPoints+factoryNumber*factoryPoints;
}


void GameControl::run()
{
	while(timeToLive > 0 && !isQuitting)
	{
		waitConditionMutex.lock();
		timeout.wait(&waitConditionMutex, 1000);
		waitConditionMutex.unlock();
		updateGamePoints();
		timeToLive--;
		emit updatedTimeAndPoints(timeToLive, points);
	}
	if(!isQuitting) emit endGame();
}

void GameControl::quitNow()
{
	waitConditionMutex.lock();
	timeout.wakeAll();
	waitConditionMutex.unlock();
	isQuitting = true;
}


