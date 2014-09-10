#include <iostream>
#include <vector>
#include "opencv2/opencv.hpp"
#include <fstream>

#include "ros/ros.h"
#include "sensor_msgs/CompressedImage.h"

#include "Zone.h"

class LittleObject
{
	public:
		//costruttore
		LittleObject(bool from_ros);
		//carica l'immagine da file
		void getImg(const char* file);
		//carica l'immagine da ros
		void getImgRos(const sensor_msgs::CompressedImage::ConstPtr& message);
		//aggiorna l'immagine
		void updateImg();
		//salva nei vettori i colori selezionati 
		void getColor(char color);
		//elimina i duplicati
		void eliminateDuplicates(char color);
		//sceglie il vettore su cui operare
		std::vector<cv::Vec3b>* selectVector(char color);
		//stampa su file
		void printOnfile(char color, char c, std::ofstream& output);
		//stampa il numero di elementi in testa al filename
		void printOnfileNumber(std::ofstream& output);
		//visualizza l'immagine caricata
		char showImage();
		//svuota i vettori
		void cleanVectors();
		//immagine
		cv::Mat img;
		//zona 
		Zone* Z;
	private:
		//vettore r
		std::vector<cv::Vec3b> r;
		//vettore b
		std::vector<cv::Vec3b> b;
		//vettore g
		std::vector<cv::Vec3b> g;
		//modalità
		bool mode_r;
		//filename
		const char* path;
};
