/*
 * RTCards.cpp
 *
 *  Created on: 31/lug/2012
 *      Author: dave
 */

#include "RTCards.h"
#include <string>

RTCards::RTCards() :
		QVBoxLayout()
{
	label = new QLabel("Carte Attive");
	addWidget(label);
	cardGrid = new QGridLayout();
	for (int i = 0; i < ROWS * COLS; i++)
	{
		RTCard* card = new RTCard(i);
		cardGrid->addWidget(card,i%ROWS, i/ROWS, 1,1);
		cardList.push_back(card);
	}
	addLayout(cardGrid);
}

void RTCards::setCardStatus(int cardNumber, bool cardStatus)
{
	cardList[cardNumber]->setCardStatus(cardStatus);
}

RTCard::RTCard(int number) : QLabel()
{
	QString labelText = QString::number(number);
	this->setText(labelText);
	setCardStatus(true);
}
void RTCard::setCardStatus(bool isActive)
{
	QColor color = (isActive) ? Qt::blue : Qt::red;
	QPalette Pal(this->palette());
	Pal.setColor(QPalette::Window, color);
	setPalette(Pal);
	setAutoFillBackground(true);
}

