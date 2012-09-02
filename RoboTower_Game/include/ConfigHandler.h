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

#ifndef RTCONFIGHANDLER_H_
#define RTCONFIGHANDLER_H_

#include <QXmlDefaultHandler>
#include <vector>

typedef struct rt_config
{
	int timeToLive;
	int setupTime;
	int towerPoints;
	int factoryPoints;
	int towerId;
	int factories;
	int towerRechargeIncrement;
	int factoryRechargeIncrement;
} ConfigGeneral;

typedef struct rt_config_rfid
{
	std::string id;
	int num;
	std::string action;
} ConfigRfidEntry;

class ConfigHandler: public QXmlDefaultHandler
{

private:
	QString currentAction;
	ConfigGeneral configuration;
	std::vector<std::vector<ConfigRfidEntry> > rfids;
public:
	ConfigHandler();
	bool startElement(const QString& namespaceURI, const QString& localName,
			const QString& qName, const QXmlAttributes& atts);
	ConfigGeneral getMainConfiguration();
	std::vector<std::vector<ConfigRfidEntry> > getRfidList();
};

#endif /* RTCONFIGHANDLER_H_ */
