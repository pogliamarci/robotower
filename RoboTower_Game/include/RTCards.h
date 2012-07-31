/*
 * RTCards.h
 *
 *  Created on: 31/lug/2012
 *      Author: dave
 */

#ifndef RTCARDS_H_
#define RTCARDS_H_

#include <QtGui>
#include <vector>

#define ROWS 5
#define COLS 3


class RTCard : public QLabel
{
public:
	RTCard(int number);
	void setCardStatus(bool cardStatus);
};

class RTCards : public QVBoxLayout
{
private:
	QLabel* label;
	QGridLayout* cardGrid;
	std::vector<RTCard*> cardList;
public:
	RTCards();
	void setCardStatus(int cardNumber, bool cardStatus);
};

#endif /* RTCARDS_H_ */
