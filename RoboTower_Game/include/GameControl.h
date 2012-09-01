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

typedef struct rfid_entry
{
	int number;
	int status;
	std::string action;
} RfidEntry;

enum GameStatus
{
	STARTED, STOPPED, PAUSED, WAITING
};

class GameControl: public QThread
{
Q_OBJECT

private:
	const int gameMaxTime;
	const int gameSetupTime;
	const int towerPoints;
	const int factoryPoints;
	const int mainTower;
	const int towersNumber;
private:
	int timeToLive;
	int timeToStart;
	int score;
	bool* towers;
	bool isQuitting;
	int cardRecharge;
	std::map<std::string, RfidEntry> rfidMap;
	std::queue<std::string> disabledRfid;
	QMutex waitConditionMutex;
	QWaitCondition timeout;
	GameStatus status;
	GameHistory history;

public:
	GameControl(GameConfiguration config);
	~GameControl();
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
		for (int i = 0; i < towersNumber; i++)
			if (towers[i] && i != mainTower)
				count++;
		return count;
	}
	inline int getTowerNumber()
	{
		return towers[mainTower - 1] ? 1 : 0;
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
	void initializeRfidConfiguration(GameConfiguration config);
	void updateGamePoints();
	void rechargeCard();
	void resetRound();
	void resetRFID();
	void performMatchOneStepUpdate();
	void wakeup();
	void resetTowers();

signals:
	void updatedTimeAndPoints(int timeToLive, int score); //emitted at the end of each iteration
	void updatedRfidStatus(int rfid, bool status); //emitted when RFID status changes
	void rfidActionNotification(std::string id);
	void endGame(int won, int lost, int score); //emitted when the game ends
	void towersUpdate(int factoriesNumber, int towersNumber);
	void robotIsEnabled(bool enabled);
	void updateRemainingTime(int remainingTime);
};

#endif /* GAMECONTROL_H_ */
