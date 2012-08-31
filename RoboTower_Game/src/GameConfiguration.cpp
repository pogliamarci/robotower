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

#include "GameConfiguration.h"

#include <iostream>
using namespace std;

GameConfiguration::GameConfiguration(std::string filePath)
{
	RTConfigHandler handler;
	QString path = QString::fromStdString(filePath);
	QFile file(path);

	QXmlInputSource inputSource(&file);
	QXmlSimpleReader reader;
	reader.setContentHandler(&handler);
	reader.parse(inputSource);
	configuration = handler.getMainConfiguration();
	rfidList = handler.getRfidList();

	cout << "Configuration parsed" << endl;
	cout << "timetolive : " << configuration.timetolive << endl;
	cout << "setuptime : " << configuration.setuptime << endl;
	cout << " tower : " << configuration.tower << endl;
	cout << " factory: " << configuration.factory << endl;
	cout << " towerid: " << configuration.towerid << endl;
	cout << " factories: " << configuration.factories << endl;
	cout << endl << endl;
	cout << "RFID" << endl;
	for(int i = 0; i < rfidList.size(); i++)
	{
		cout << "id: " << rfidList.at(i).id << endl;
		cout << "num: " << rfidList.at(i).num << endl;
		cout << "action: " << rfidList.at(i).action << endl;
	}

}

RTConfigGeneral GameConfiguration::getMainConfiguration()
{
	return this->configuration;
}

int GameConfiguration::getNumActions() {
	return rand() % 5 + 1;
}

std::vector<int> GameConfiguration::getRfidList(int actionId) {
	std::vector<int> ret;
	int x = rand() % 8 + 1;
	for(int i = 0; i < x; i++) {
		ret.push_back(actionId*3+i);
	}
	return ret;
}
