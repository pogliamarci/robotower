#ifndef MAINWINDOW_H_
#define MAINWINDOW_H_

#include <QtGui>

class RTMainWindow : public QMainWindow
{
	private:
		QToolBar* fileToolBar;
		QAction* newGameAction;
		QHBoxLayout* mainLayout;
		QGridLayout* leftLayout;
		QWidget* mainWidget;
	public:
		RTMainWindow(QWidget* parent = 0);
	private:
		void setupToolbar();
		void setupLayout();
};

#endif /* MAINWINDOW_H_ */
