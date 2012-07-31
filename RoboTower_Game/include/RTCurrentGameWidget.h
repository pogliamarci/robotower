/*
 * CurrentGameWidget.h
 *
 *  Created on: 31 Jul 2012
 *      Author: marcello
 */

#ifndef RTCURRENTGAMEWIDGET_H_
#define RTCURRENTGAMEWIDGET_H_

#include <QtGui>

class RTCurrentGameWidget : public QGroupBox
{
private:
	QGridLayout* theLayout;
	QPushButton* pauseBtn;
	QLCDNumber* currentScore;
	QLCDNumber* currentTTL;
	QGroupBox* goalsBox;
	QGridLayout* innerGoalsLayout;
	QLabel* towersCnt;
	QLabel* factoriesCnt;
public:
	RTCurrentGameWidget(QWidget* parent = 0);
	void updateCounter(int towers, int factories);
	void updateScore(int newScore);
	void updateTimer(int newTtl);
};


#endif /* CURRENTGAMEWIDGET_H_ */
