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

#ifndef CONFIGURATIONFACTORY_H_
#define CONFIGURATIONFACTORY_H_

#include <QFile>
#include <QString>
#include <QXmlAttributes>

#include "GameConfiguration.h"

class ConfigurationFactory: public QXmlDefaultHandler
{
public:
	static GameConfiguration loadConfiguration(QString relativePath);
	static GameConfiguration loadConfiguration(QString folder, QString relativePath);
	bool startElement(const QString& namespaceURI,
				const QString& localName, const QString& qName,
				const QXmlAttributes& atts);
private:
	ConfigurationFactory();

private:
	GameConfiguration::GeneralData configuration;
	QString currentAction;
	std::vector<std::vector<GameConfiguration::RfidEntry> > rfidList;
};

#endif /* CONFIGURATIONFACTORY_H_ */
