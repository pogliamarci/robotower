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

#include <vector>
#include <string>

class GameConfiguration
{
public:
	struct RfidEntry
	{
		std::string id;
		int num;
		std::string action;
	};
	struct GeneralData
	{
		int timeToLive;
		int setupTime;
		int towerPoints;
		int factoryPoints;
		int towerId;
		int factories;
		int basicRechargeIncrement;
		int factoryRechargeIncrement;
	};
private:
	GeneralData configuration;
	std::vector<std::vector<RfidEntry> > rfidList;

public:
	GameConfiguration(GeneralData configuration,
			std::vector<std::vector<RfidEntry> > rfidList)
	{
		this->configuration = configuration; //FIXME copia?
		this->rfidList = rfidList;
	}
	inline int getGameMaxTime()
	{
		return configuration.timeToLive;
	}
	inline int getGameSetupTime()
	{
		return configuration.setupTime;
	}
	inline int getTowerPoints()
	{
		return configuration.towerPoints;
	}
	inline int getFactoryPoints()
	{
		return configuration.factoryPoints;
	}
	inline int getMainTower()
	{
		return configuration.towerId;
	}

	inline int getTowersNumber()
	{
		return configuration.factories + 1;
	}

	inline int getBasicRechargeIncrement()
	{
		return configuration.basicRechargeIncrement;
	}

	inline int getFactoryRechargeIncrement()
	{
		return configuration.factoryRechargeIncrement;
	}

	inline int getNumActions()
	{
		return rfidList.size();
	}

	inline std::vector<RfidEntry> getRfidList(int actionId)
	{
		return rfidList.at(actionId);
	}

};

#endif /* GAMECONFIGURATION_H_ */
