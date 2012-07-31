#include "RTCurrentGameWidget.h"

RTCurrentGameWidget::RTCurrentGameWidget(QWidget* parent) : QGroupBox(parent)
{
	theLayout = new QGridLayout();
	pauseBtn = new QPushButton("Pause");
	currentScore = new QLCDNumber();
	currentTTL = new QLCDNumber();
	goalsBox = new QGroupBox();
	towersCnt = new QLabel("0");
	factoriesCnt = new QLabel("0");
	innerGoalsLayout = new QGridLayout();

	setTitle("Current Game");

	/* inner goals layout */
	innerGoalsLayout->addWidget(new QLabel("Towers: "), 1, 1, 1, 1);
	innerGoalsLayout->addWidget(towersCnt, 1, 2, 1, 1);
	innerGoalsLayout->addWidget(new QLabel("Factories: "), 2, 1, 1, 1);
	innerGoalsLayout->addWidget(factoriesCnt, 2, 2, 1, 1);
	goalsBox->setLayout(innerGoalsLayout);

	/* main layout */
	theLayout->addWidget(pauseBtn, 1, 1, 1, 1);
	theLayout->addWidget(new QLabel("Score:"), 1, 2, 1, 1);
	theLayout->addWidget(currentScore, 1, 3, 1, 2);
	theLayout->addWidget(goalsBox, 2, 1, 3, 2);
	theLayout->addWidget(new QLabel("Time to live:"), 2, 3, 1, 2);
	theLayout->addWidget(currentTTL, 3, 3, 2, 2);
	setLayout(theLayout);

}

void RTCurrentGameWidget::updateCounter(int towers, int factories)
{
	towersCnt->setText(QString::number(towers));
	factoriesCnt->setText(QString::number(factories));
}

void RTCurrentGameWidget::updateScore(int newScore)
{
	currentScore->display(newScore);
}

void RTCurrentGameWidget::updateTimer(int newTtl)
{
	currentTTL->display(newTtl);
}
