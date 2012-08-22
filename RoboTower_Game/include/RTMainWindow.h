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
#include "RTCurrentGameWidget.h"
#include "RTCards.h"
#include "RTPopupTimer.h"

class RTMainWindow: public QMainWindow
{
	Q_OBJECT
private:
	QToolBar* fileToolBar;
	QAction* newGameAction;
	QHBoxLayout* mainLayout;
	QGridLayout* leftLayout;
	QWidget* mainWidget;
	/* internal widgets */
	RTCurrentGameWidget* currentGame;
	RTCards* cardsLayout;
	/* buttons (center left) */
	QVBoxLayout* btnLayout;
	QPushButton* startBtn;
	QPushButton* stopBtn;
	/* Stats */
	QGridLayout* statsLayout;
	QGroupBox* statsGroupBox;
	QLabel* statWon;
	QLabel* statTotalScore;
	QLabel* statLost;
	/* PopUp*/
	RTPopupTimer* popupTimer;

public:
	RTMainWindow(QWidget* parent = 0);
	~RTMainWindow();
public slots:
	void updateTowers(int factoryNumber, int towersNumber); //Updates tower counter
	void updateData(int timeToLive, int score); //Updates time to live and score
	void updateCardStatus(int card, bool status); //Updates Rfid status
	void updateHistory(int won, int lost, int score);
	void updateSetupPopup(int remainingTime);
private:
	void setupButtons();
	void setupStats();
	void setupToolbar();
	void setupLayout();
	void setButtonStatus(bool isRunning);
private slots:
	void startOnClick();
	void stopOnClick();
	void newGameClicked();
signals:
	void start();
	void stop();
	void togglePause();
	void newGame();
};

#endif /* MAINWINDOW_H_ */
