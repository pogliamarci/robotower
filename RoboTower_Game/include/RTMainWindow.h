#ifndef MAINWINDOW_H_
#define MAINWINDOW_H_

#include <QtGui>
#include "RTCurrentGameWidget.h"

class RTMainWindow : public QMainWindow
{
	private:
		QToolBar* fileToolBar;
		QAction* newGameAction;
		QHBoxLayout* mainLayout;
		QGridLayout* leftLayout;
		QWidget* mainWidget;
		/* internal widgets */
		RTCurrentGameWidget* currentGame;
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
	private:
		void setupButtons();
		void setupStats();
		void setupToolbar();
		void setupLayout();
};

#endif /* MAINWINDOW_H_ */
