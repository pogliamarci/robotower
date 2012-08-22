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

#include <QtGui>
#include <vector>

#define ROWS 5
#define COLS 3

class RTCard: public QLabel
{
Q_OBJECT
public:
	RTCard(int number);
	void setCardStatus(bool cardStatus);
private:
	void setTextWhite();
};

class RTCards: public QGridLayout
{
Q_OBJECT
private:
	QLabel* label;
	QGridLayout* cardGrid;
	std::vector<RTCard*> cardList;
public:
	RTCards();
	void setCardStatus(int cardNumber, bool cardStatus);
	void setGeometry(const QRect& rect);
private:
	void addCards();
};

#endif /* RTCARDS_H_ */
