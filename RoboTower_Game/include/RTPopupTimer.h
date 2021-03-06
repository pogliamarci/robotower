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

#ifndef RTPOPUPTIMER_H_
#define RTPOPUPTIMER_H_

#include <QtGui>

class RTPopupTimer: public QWidget
{
Q_OBJECT
private:
	QLabel* text;

protected:
	void closeEvent(QCloseEvent* e);
	void keyPressEvent(QKeyEvent *e);
public:
	RTPopupTimer(QWidget* parent = 0);
	void update(int remainingTime);
signals:
	void closed();
};

#endif /* RTPOPUPTIMER_H_ */
