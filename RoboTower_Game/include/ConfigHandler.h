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

class ConfigHandler: public QXmlDefaultHandler
{
public:
	struct GeneralData
	{
		int timeToLive;
		int setupTime;
		int towerPoints;
		int factoryPoints;
		int towerId;
		int factories;
		int towerRechargeIncrement;
		int factoryRechargeIncrement;
	};

	struct RfidEntry
	{
		std::string id;
		int num;
		std::string action;
	};
private:
	QString currentAction;
	GeneralData configuration;
	std::vector<std::vector<RfidEntry> > rfids;
public:

	ConfigHandler();
	bool startElement(const QString& namespaceURI, const QString& localName,
			const QString& qName, const QXmlAttributes& atts);
	GeneralData getMainConfiguration();
	std::vector<std::vector<RfidEntry> > getRfidList();
};

#endif /* RTCONFIGHANDLER_H_ */
