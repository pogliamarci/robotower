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

#ifndef GAMECONTROL_H_
#define GAMECONTROL_H_

#include <QThread>
#include <QObject>
#include <QMutex>
#include <QWaitCondition>

#include <string>
#include <map>
#include <queue>

#include "GameHistory.h"
#include "GameConfiguration.h"

class GameControl: public QThread
{
Q_OBJECT

private:
	struct RfidEntry
	{
		int number;
		int status;
		std::string action;
	};

	enum GameStatus
	{
		STARTED, STOPPED, PAUSED, WAITING
	};
	const int gameMaxTime;
	const int gameSetupTime;
	const int towerPoints;
	const int factoryPoints;
	const int mainTower;
	const int towerRechargeIncrement;
	const int factoryRechargeIncrement;
	static const int maxRedLeds = 4;
private:
	int timeToLive;
	int timeToStart;
	int score;
	std::vector<bool> towers;
	bool isQuitting;
	int cardRecharge;
	std::map<std::string, RfidEntry> rfidMap;
	std::queue<std::string> disabledRfid;
	QMutex waitConditionMutex;
	QMutex timeMutex;
	QWaitCondition timeout;
	GameStatus status;
	GameHistory history;

public:
	GameControl(GameConfiguration& config);
	void run();
	inline int getTimeToLive()
	{
		return timeToLive;
	}
	inline int getPoints()
	{
		return score;
	}
	inline int getFactoryNumber()
	{
		int count = 0;
		for (size_t i = 0; i < towers.size(); i++)
			if (towers.at(i) && i != ((size_t) mainTower - 1))
				count++;
		return count;
	}
	inline int getTowerNumber()
	{
		return towers.at(mainTower - 1) ? 1 : 0;
	}
public slots:
	void manageRfid(std::string id);
	void updateTowers(int towerNumber);
	void quitNow();

	void startGame();
	void stopGame();
	void togglePause();
	void resetGame();
private:
	void initializeRfidConfiguration(GameConfiguration& config);
	void updateTime(int increment  = -1);
	void updateGamePoints();
	void rechargeCard();
	void resetRound();
	void resetRFID();
	void performMatchOneStepUpdate();
	void wakeup();
	void resetTowers();
	int ledsFromTime();

signals:
	void timeIsUpdated(int timeToLive);
	void pointsAreUpdated(int score); //emitted at the end of each iteration
	void updatedRfidStatus(int rfid, bool status); //emitted when RFID status changes
	void rfidActionNotification(std::string id);
	void endGame(int won, int lost, int score); //emitted when the game ends
	void towersUpdate(int factoriesNumber, int towersNumber);
	void robotIsEnabled(bool enabled);
	void mustSetLeds(int numLeds);
	void updateRemainingTime(int remainingTime);
};

#endif /* GAMECONTROL_H_ */
