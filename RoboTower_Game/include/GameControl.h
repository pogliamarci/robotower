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

#include <fstream>
#include <string>
#include <map>
#include <queue>

#include "GameHistory.h"

typedef struct rfid_entry
{
	int number;
	int status;
} RfidEntry;

enum GameStatus {STARTED, STOPPED, PAUSED};

class GameControl: public QThread
{
Q_OBJECT
private:
	int timeToLive;
	int score;
	int factoryNumber;
	int towerNumber;
	bool isQuitting;
	int cardRecharge;
	std::map<std::string, RfidEntry> rfidMap;
	std::queue<std::string> disabledRfid;
	QMutex waitConditionMutex;
	QWaitCondition timeout;
	GameStatus status;
	GameHistory* history;
private:
	static const int gameMaxTime = 300;
	static const int towerPoints = 100;
	static const int factoryPoints = 20;

public:
	GameControl(int factoryNumber, int towerNumber);
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
		return factoryNumber;
	}
	inline int getTowerNumber()
	{
		return towerNumber;
	}
private:
	void initializeRfidConfiguration(std::string configFile);
	void populateMapWithLine(std::string configLine, int index);
	void updateGamePoints();
	void rechargeCard();
	void resetRound();

public slots:
	void disableRFID(std::string id);
	void updateTowers(int factoryNumber, bool destroyedTower);
	void quitNow();

	void startGame();
	void stopGame();
	void togglePause();
	void resetGame();
signals:
	void updatedTimeAndPoints(int timeToLive, int score); //emitted at the end of each iteration
	void updatedRfidStatus(int rfid, bool status); //emitted when RFID status changes
	void hasToResetRobot(); //emitted when the robot should be resetted
	void endGame(); //emitted when the game ends
};

#endif /* GAMECONTROL_H_ */
