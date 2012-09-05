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

#ifndef RTCURRENTGAMEWIDGET_H_
#define RTCURRENTGAMEWIDGET_H_

#include <QtGui>

class RTCurrentGameWidget : public QGroupBox
{
	Q_OBJECT
private:
	QPushButton* pauseBtn;
	QLCDNumber* currentScore;
	QLCDNumber* currentTTL;
	QLabel* towersCnt;
	QLabel* factoriesCnt;
	bool isPaused;
public:
	RTCurrentGameWidget(QWidget* parent = 0);
	void updateCounter(int towers, int factories);
	void updateScore(int newScore);
	void updateTimer(int newTtl);
	void setPauseEnabled(bool isPauseEnabled);
private slots:
	void onPauseClick();
signals:
	void togglePause();
};


#endif /* CURRENTGAMEWIDGET_H_ */
