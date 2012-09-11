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

#include "ConfigurationFactory.h"

#include <QCoreApplication>
#include <QXmlInputSource>
#include <QXmlSimpleReader>


GameConfiguration ConfigurationFactory::loadConfiguration(QString relativePath)
{
	return loadConfiguration(QCoreApplication::applicationDirPath() ,relativePath);
}

GameConfiguration ConfigurationFactory::loadConfiguration(QString folder, QString relativePath)
{
	QFile file(folder + relativePath);
	QXmlInputSource inputSource(&file);
	QXmlSimpleReader reader;
	ConfigurationFactory* handler = new ConfigurationFactory();
	reader.setContentHandler(handler);
	reader.parse(inputSource);
	return GameConfiguration(handler->configuration, handler->rfidList);
}

bool ConfigurationFactory::startElement(const QString& namespaceURI,
		const QString& localName, const QString& qName,
		const QXmlAttributes& atts)
{
	if (localName == "time")
	{
		configuration.timeToLive = atts.value("timetolive").toInt();
		configuration.setupTime = atts.value("setuptime").toInt();
	}
	else if (localName == "points")
	{
		configuration.towerPoints = atts.value("tower").toInt();
		configuration.factoryPoints = atts.value("factory").toInt();
	}
	else if (localName == "goals")
	{
		configuration.towerId = atts.value("towerid").toInt();
		configuration.factories = atts.value("factories").toInt();
	}
	else if (localName == "action")
	{
		QString action = atts.value("name");
		if (action != currentAction)
		{
			std::vector<GameConfiguration::RfidEntry> actionGroup;
			rfidList.push_back(actionGroup);
			currentAction = action;
		}
	}
	else if (localName == "tag")
	{
		GameConfiguration::RfidEntry entry;
		entry.id = atts.value("id").toStdString();
		entry.num = atts.value("num").toInt();
		entry.action = currentAction.toStdString();
		rfidList.back().push_back(entry);
	}
	else if (localName == "recharge")
	{
		configuration.factoryRechargeIncrement = atts.value("factory").toInt();
		configuration.basicRechargeIncrement = atts.value("basic").toInt();
	}
	return true;
}

ConfigurationFactory::ConfigurationFactory()
{
	currentAction = "";
	configuration.timeToLive = 0;
	configuration.setupTime = 0;
	configuration.towerPoints = 0;
	configuration.factoryPoints = 0;
	configuration.towerId = 0;
	configuration.factories = 0;
	configuration.basicRechargeIncrement = 0;
	configuration.factoryRechargeIncrement = 0;
}

