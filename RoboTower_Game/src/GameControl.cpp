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

GameControl::GameControl(GameConfiguration& config) :
		gameMaxTime(config.getGameMaxTime()),
		gameSetupTime(config.getGameSetupTime()),
		towerPoints(config.getTowerPoints()),
		factoryPoints(config.getFactoryPoints()),
		mainTower(config.getMainTower()),
		towerRechargeIncrement(config.getTowerRechargeIncrement()),
		factoryRechargeIncrement(config.getFactoryRechargeIncrement())
{
	towers.resize(config.getTowersNumber(), false); // FIXME what if mainTower >= towersNumber?
	timeToLive = gameMaxTime;
	score = 0;
	cardRecharge = 0;
	isQuitting = false;
	status = STOPPED;
	resetTowers();
	initializeRfidConfiguration(config);
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
		if (status != STOPPED)
			timeout.wait(&waitConditionMutex, 1000);
		locker.unlock();
		switch (status)
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
			if (timeToStart <= 0)
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

void GameControl::manageRfid(std::string id)
{
	if (status != STARTED)
		return;
	RfidEntry &entry = rfidMap[id];
	if (entry.status)
	{
		entry.status = false;
		disabledRfid.push(id);
		emit updatedRfidStatus(entry.number, entry.status);
		if (entry.action == "modify_time")
		{
			int randomTimevariation = rand() % (gameMaxTime / 5) + 10;
			int signum = rand() % 2 == 0 ? -1 : 1;
			updateTime(signum * randomTimevariation);
		}
		else
			emit rfidActionNotification(entry.action);
	}
}

void GameControl::updateTowers(int tower)
{
	if (status != STARTED)
			return;
	towers.at(tower - 1) = false;
	emit towersUpdate(getFactoryNumber(), getTowerNumber());
}

void GameControl::quitNow()
{
	wakeup();
	isQuitting = true;
}

void GameControl::startGame()
{
	if (status == STOPPED)
	{
		status = WAITING;
		emit updateRemainingTime(timeToStart);
		emit mustSetLeds(maxRedLeds);
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
	if (status == PAUSED)
	{
		status = STARTED;
		emit robotIsEnabled(true);
	}
	else if (status == STARTED)
	{
		status = PAUSED;
		emit robotIsEnabled(false);
	}
	wakeup();
}

void GameControl::resetGame()
{
	stopGame();
	history.reset();
	emit endGame(history.getWon(), history.getLost(), history.getScore());
}

void GameControl::initializeRfidConfiguration(GameConfiguration& config)
{
	for (int i = 0; i < config.getNumActions(); i++)
	{
		std::vector<GameConfiguration::RfidEntry> groupList = config.getRfidList(i);
		for (size_t j = 0; j < groupList.size(); j++)
		{
			RfidEntry entry;
			entry.action = groupList.at(j).action;
			entry.number = groupList.at(j).num;
			entry.status = true;
			rfidMap.insert(std::make_pair(groupList.at(j).id, entry));
		}
	}
}

void GameControl::updateGamePoints()
{
	score += getTowerNumber() * towerPoints
			+ getFactoryNumber() * factoryPoints;
	emit pointsAreUpdated(score);
}

void GameControl::rechargeCard()
{
	if (disabledRfid.size() > 0)
	{
		int increment = towerRechargeIncrement * getTowerNumber()
				+ factoryRechargeIncrement * getFactoryNumber();
		cardRecharge += increment;
		if (cardRecharge >= 100)
		{
			std::string card = disabledRfid.front();
			disabledRfid.pop();
			RfidEntry& entry = rfidMap[card];
			entry.status = true;
			emit updatedRfidStatus(entry.number, entry.status);
			cardRecharge = 0;
		}
	}
}

void GameControl::resetRound()
{
	resetRFID();
	resetTowers();
	timeToLive = gameMaxTime;
	timeToStart = gameSetupTime;
	score = 0;
	cardRecharge = 0;
	emit pointsAreUpdated(score);
	emit timeIsUpdated(timeToLive);
	emit mustSetLeds(0);
	emit towersUpdate(getFactoryNumber(), getTowerNumber());
}

void GameControl::resetTowers()
{
	for (size_t i = 0; i < towers.size(); i++)
	{
		towers.at(i) = true;
	}
}

void GameControl::resetRFID()
{
	while (!disabledRfid.empty())
	{
		std::string rfid = disabledRfid.front();
		disabledRfid.pop();
		rfidMap[rfid].status = true;
		emit updatedRfidStatus(rfidMap[rfid].number, rfidMap[rfid].status);
	}
}

void GameControl::performMatchOneStepUpdate()
{
	updateTime();
	updateGamePoints();
	rechargeCard();
	bool hasWon = getTowerNumber() <= 0;
	if (timeToLive <= 0 || hasWon)
	{
		stopGame();
		history.addGame(hasWon, score);
		emit endGame(history.getWon(), history.getLost(), history.getScore());
	}
}

void GameControl::wakeup()
{
	QMutexLocker locker(&waitConditionMutex);
	timeout.wakeAll();
}

void GameControl::updateTime(int increment)
{
	QMutexLocker lock(&timeMutex);
	int oldnumleds = ledsFromTime();
	timeToLive += increment;
	if (timeToLive < 0)
		timeToLive = 0;
	int newnumleds = ledsFromTime();
	if (oldnumleds != newnumleds)
	{
		emit mustSetLeds(newnumleds);
	}
	emit timeIsUpdated(timeToLive);
}

int GameControl::ledsFromTime()
{
	if (timeToLive == 0)
		return 0;
	else
	{
		int numLeds = maxRedLeds * timeToLive / gameMaxTime + 1;
		if (numLeds > maxRedLeds)
			numLeds = maxRedLeds;
		return numLeds;
	}

}
