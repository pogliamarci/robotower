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

#ifndef RTCARDS_H_
#define RTCARDS_H_

#include "GameConfiguration.h"
#include <QtGui>
#include <map>

#define ROWS 3
#define COLS 5

class RTCard: public QLabel
{
Q_OBJECT
public:
	RTCard(int number);
	void setCardStatus(bool cardStatus);
private:
	void setTextWhite();
	void paintEvent(QPaintEvent * event);
private:
	int num;
	QImage image;
};

class RTCards: public QGridLayout
{
Q_OBJECT
private:
	QLabel* label;
	QGridLayout* cardGrid;
	std::map<int, RTCard*> cardMap;
public:
	RTCards(GameConfiguration& config);
	void setCardStatus(int cardNumber, bool cardStatus);
	void setGeometry(const QRect& rect);
private:
	void addCards(GameConfiguration& config);
};

#endif /* RTCARDS_H_ */
