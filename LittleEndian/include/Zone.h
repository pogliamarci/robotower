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

#include "opencv2/opencv.hpp"

#define RED_R CV_RGB(255,0,0)
#define GREEN_R CV_RGB(0,255,0)
#define BLUE_R CV_RGB(0,0,255)
#define WHITE_R CV_RGB(255,255,255)
#define BLACK_R CV_RGB(0,0,0)

//classe per la selezione di zone in una immagine
class Zone
{
	public:
		//costruttore
		Zone(const char* Window, cv::Mat& img);
		//distruttore
		~Zone();
		//setta il punto iniziale del
		void setStart(int x, int y);
		//setta il punto finale del rettangolo
		void setEnd(int x, int y);
		//disegna il rettangolo nella finestra
		void drawZone();
		//disegna il rettangolo mentre trascini il mouse
		void drawZone(int x, int y);
		//stampa a schermo il colore del punto (x,y)
		void pointRGB(int x, int y);
		//segna permanentemente la zona colorata
		void printZone(cv::Scalar C);
		//aggiorna l'immagine 
		void updateImg(cv::Mat& img);
		//punto iniziale del rettangolo
		cv::Point Start;
		//punto finale del rettangolo
		cv::Point End;
	private:
		bool Drag_Drop;
		const char* W;
		cv::Mat I;
};