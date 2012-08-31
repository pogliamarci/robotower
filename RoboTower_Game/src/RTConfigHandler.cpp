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

#include "RTConfigHandler.h"
#include <iostream>
using namespace std;

RTConfigHandler::RTConfigHandler() {
	configuration = {0,0,0,0,0,0};
}

bool RTConfigHandler::startElement (const QString& namespaceURI, const QString& localName,
			const QString& qName, const QXmlAttributes& atts)
{
	cout << "Buongiorno" << endl;
	if(localName == "time") {
		cout << "Ho trovato time" << endl;
		configuration.timetolive = atts.value("timetolive").toInt();
		configuration.setuptime = atts.value("setuptime").toInt();
	} else if(localName=="points") {
		cout << "Ho trovato points" << endl;
		configuration.tower = atts.value("tower").toInt();
		configuration.factory = atts.value("factory").toInt();
	} else if(localName=="goals") {
		cout << "Ho trovato towerid" << endl;
		configuration.towerid = atts.value("towerid").toInt();
		configuration.factories = atts.value("factories").toInt();
	} else if(localName=="tag") {
		cout << "Ho trovato tag" << endl;
		RTConfigRfidEntry entry;
		entry.id = atts.value("id").toStdString();
		entry.num = atts.value("num").toInt();
		entry.action = atts.value("action").toStdString();
		rfids.push_back(entry);
	}
	return true;
}

RTConfigGeneral RTConfigHandler::getMainConfiguration()
{
	return configuration;
}

std::vector<RTConfigRfidEntry> RTConfigHandler::getRfidList()
{
	return rfids;
}
