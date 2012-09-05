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

#ifndef MAINWINDOW_H_
#define MAINWINDOW_H_

#include <QtGui>

#include "GameConfiguration.h"
#include "SoundManager.h"

class RTCurrentGameWidget;
class RTCards;
class RTPopupTimer;

class RTMainWindow: public QMainWindow
{
	Q_OBJECT
private:
	RTCurrentGameWidget* currentGame;
	RTCards* cardsLayout;
	QAction* newGameAction;
	QPushButton* startBtn;
	QPushButton* stopBtn;
	/* Stats */
	QLabel* statWon;
	QLabel* statTotalScore;
	QLabel* statLost;
	/* PopUp*/
	RTPopupTimer* popupTimer;
	bool popupNeedsRebuilding;
	SoundManager soundmanager;

public:
	RTMainWindow(GameConfiguration& config, QWidget* parent = 0);
	~RTMainWindow();
public slots:
	void updateTowers(int factoryNumber, int towersNumber);
	void updateTime(int timeToLive);
	void updatePoints(int score);
	void updateCardStatus(int card, bool status);
	void updateHistory(int won, int lost, int score);
	void updateSetupPopup(int remainingTime);
private:
	QLayout* setupButtons();
	QWidget* setupStats();
	void setupToolbar();
	void setupLayout(GameConfiguration& config);
	void setButtonStatus(bool isRunning);
private slots:
	void startOnClick();
	void stopOnClick();
	void newGameClicked();
	void buildSetupPopup();

signals:
	void start();
	void stop();
	void togglePause();
	void newGame();
};

#endif /* MAINWINDOW_H_ */
