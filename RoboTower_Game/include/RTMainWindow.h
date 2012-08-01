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

public:
	RTMainWindow(QWidget* parent = 0);
public slots:
	void updateTowers(int factoryNumber, bool destroyedTower); //Updates tower counter
	void updateData(int timeToLive, int score); //Updates time to live and score
	void updateCardStatus(int card, bool status); //Updates Rfid status
private:
	void setupButtons();
	void setupStats();
	void setupToolbar();
	void setupLayout();
};

#endif /* MAINWINDOW_H_ */
