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
#include <QMutexLocker>
#include "GameControl.h"

GameControl::GameControl(int factoryNumber, int towerNumber)
{
	timeToLive = gameMaxTime;
	score = 0;
	cardRecharge = 0;
	isQuitting = false;
	status = STOPPED;
	this->factoryNumber = factoryNumber;
	this->towerNumber = towerNumber;
	history = new GameHistory();
	initializeRfidConfiguration("../../rfidconfig.txt");
}

void GameControl::run()
{
	while (!isQuitting)
	{
		QMutexLocker locker(&waitConditionMutex);
		// We should wait one second (that is needed both from
		// the waiting and running states), and AFTER THAT make the relevant
		// checks (if the game is stopped or paused during the wait
		// timer don't get decremented...)
		if(status != STOPPED) timeout.wait(&waitConditionMutex, 1000);
		locker.unlock();
		switch(status)
		{
		case STOPPED:
			resetRound();
		case PAUSED:
			locker.relock();
			timeout.wait(&waitConditionMutex);
			break;
		case WAITING:
			timeToStart--;
			emit updateRemainingTime(timeToStart);
			if (timeToStart <= 0) // enable the robot when timeToStart == -1,
					// (GUI expects timeToStart == 0 to hide the dialog window)
			{
				emit robotIsEnabled(true);
				status = STARTED;
			}
			break;
		case STARTED:
			performMatchOneStepUpdate();
			break;
		default:
			std::cerr << "BUG!" << std::endl;
			abort();
			break;
		}
	}
}

void GameControl::disableRFID(std::string id)
{
	RfidEntry &entry = rfidMap[id];
	if (entry.status)
	{
		entry.status = false;
		disabledRfid.push(id);
		emit updatedRfidStatus(entry.number, entry.status);
	}
}

void GameControl::updateTowers(int factories, int towers)
{
	this->factoryNumber = factories;
	this->towerNumber = towers;
	emit towersUpdate(factoryNumber, towerNumber);
}

void GameControl::quitNow()
{
	wakeup();
	isQuitting = true;
}

void GameControl::startGame()
{
	if(status == STOPPED)
	{
		status = WAITING;
		emit updateRemainingTime(timeToStart);
		wakeup();
	}
}

void GameControl::stopGame()
{
	status = STOPPED;
	wakeup();
	emit robotIsEnabled(false);
}

void GameControl::togglePause()
{
	if(status==PAUSED)
	{
		status = STARTED;
		emit robotIsEnabled(true);
	}
	else if(status==STARTED)
	{
		status = PAUSED;
		emit robotIsEnabled(false);
	}
	wakeup();
}

void GameControl::resetGame()
{
	stopGame();
	delete history;
	history = new GameHistory();
	emit endGame(history->getWon(),
			history->getLost(), history->getScore());
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
		RfidEntry entry =
		{ index, true };
		rfidMap.insert(std::make_pair(id, entry));
	}
}

void GameControl::updateGamePoints()
{
	score += towerNumber * towerPoints + factoryNumber * factoryPoints;
}

void GameControl::rechargeCard()
{
	if (disabledRfid.size() > 0)
	{
		int increment = 100 / (30 - 5 * factoryNumber);
		cardRecharge += increment;
		if (cardRecharge >= 100)
		{
			std::string card = disabledRfid.front();
			disabledRfid.pop();
			RfidEntry entry = rfidMap[card];
			entry.status = true;
			emit updatedRfidStatus(entry.number, entry.status);
			cardRecharge = 0;
		}
	}
}

void GameControl::resetRound()
{
	resetRFID();
	timeToLive = gameMaxTime;
	timeToStart = gameSetupTime;
	score = 0;
	cardRecharge = 0;
	emit updatedTimeAndPoints(timeToLive, score);
	emit towersUpdate(factoryNumber, towerNumber);
	emit hasToResetRobot();
}

void GameControl::resetRFID()
{
	while(!disabledRfid.empty())
	{
		std::string rfid = disabledRfid.front();
		disabledRfid.pop();
		rfidMap[rfid].status = true;
		emit updatedRfidStatus(rfidMap[rfid].number, rfidMap[rfid].status);
		emit rfidEnableNotification(rfid);
	}
}

void GameControl::performMatchOneStepUpdate()
{
	updateGamePoints();
	timeToLive--;
	rechargeCard();
	emit updatedTimeAndPoints(timeToLive, score);
	if(timeToLive <= 0 || towerNumber == 0)
	{
		stopGame();
		bool hasWon = towerNumber > 0;
		history->addGame(hasWon, score);
		emit endGame(history->getWon(),
				history->getLost(), history->getScore());
	}
}

void GameControl::wakeup()
{
	QMutexLocker locker(&waitConditionMutex);
	timeout.wakeAll();
}

GameControl::~GameControl()
{
	delete history;
}
