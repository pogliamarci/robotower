#include "RTMainWindow.h"
#include "RTCards.h"

void RTMainWindow::setupToolbar()
{
	fileToolBar = addToolBar(tr("MainBar"));
	newGameAction = new QAction(tr("&New game"), this);
	fileToolBar->addAction(newGameAction);
	addToolBar(Qt::TopToolBarArea, fileToolBar);
}

void RTMainWindow::setupLayout()
{
	mainLayout = new QHBoxLayout();
	leftLayout = new QGridLayout();
	mainWidget = new QWidget();
	setCentralWidget(mainWidget);
	mainWidget->setLayout(mainLayout);
	mainLayout->addLayout(leftLayout);
	mainLayout->addLayout(new RTCards());
	leftLayout->addWidget(new QPushButton("Punteggio"), 1, 1, 1, 4);
	leftLayout->addWidget(new QPushButton("Bottoni"), 2, 1, 2, 1);
	leftLayout->addWidget(new QPushButton("Time to live"), 2, 2, 2, 3);
	leftLayout->addWidget(new QPushButton("Torri e fabbriche"), 4, 1, 1, 2);
	leftLayout->addWidget(new QPushButton("Risultati totali"), 4, 3, 1, 2);
}

RTMainWindow::RTMainWindow(QWidget* parent) : QMainWindow(parent)
{
	setupToolbar();
	setupLayout();
}
