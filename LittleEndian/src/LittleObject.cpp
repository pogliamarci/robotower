#include "LittleObject.h"

using namespace std;
using namespace cv;

//costruttore della classe. fa poca roba
LittleObject::LittleObject(bool from_ros)
{
	if(from_ros)
	{
		cout << "carico le immagini da ros" << endl;
		this->mode_r = true;
	}
	else
	{
		cout << "carico immagine da file" << endl;
		this->mode_r = false;
	}
}

//funzione inline per la selezione del vettore
inline vector<Vec3b>* LittleObject::selectVector(char color)
{
	switch(color)
	{
		case 'r':
			return &(this->r);
			break;
		case 'g':
			return &(this->g);
			break;
		case 'b':
			return &(this->b);
			break;
		default:
		    return NULL;
	}
}

//callback per ricevere i messaggi da ROS
void LittleObject::getImgRos(const sensor_msgs::CompressedImage::ConstPtr& message)
{
	this->img = imdecode(message->data, CV_LOAD_IMAGE_ANYCOLOR);
	Z->updateImg(img);
}

//metodo per caricare le immagini da file
void LittleObject::getImg(const char* file)
{
	this->path = file;
	this->img = imread(file,CV_LOAD_IMAGE_COLOR);
	if(img.empty())
	{
		cerr << "Errore: File non valido o non esistente" << endl;
		exit(EXIT_FAILURE);
	}
}

//metodo per aggiornare l'immagine
void LittleObject::updateImg()
{
	if(!mode_r)
		this->getImg(this->path);
}

//metodo per prendere il colore rgb dei pixel selezionati
void LittleObject::getColor(char color)
{
	int x,y;
	vector<Vec3b> *V;
	V=selectVector(color);
	for(x=Z->Start.x; x<Z->End.x; x++)
		for(y=Z->Start.y; y<Z->End.y; y++)
		{
			V->push_back(this->img.at<Vec3b>(y, x));
		}
}

//metodo per eliminare i duplicati dai vettori dei colori
void LittleObject::eliminateDuplicates(char color)
{
	vector<Vec3b>* V;
	vector<Vec3b>::iterator i1,i2;
	V=selectVector(color);
	for(i1=V->begin(); i1<V->end(); i1++)
		for(i2=(i1+1); i2<V->end(); i2++)
		{
			if(((*i2)[0]==(*i1)[0]) && ((*i2)[1]==(*i1)[1]) && ((*i2)[2]==(*i1)[2]) ) 
			{
				i2=V->erase(i2);
				i2--;
			}
		}
}

//metodo per stampare il numero di campioni
void LittleObject::printOnfileNumber(ofstream& output)
{
	output << (this->r.size()+this->g.size()+this->b.size());
}

//metodo per stampare i campioni di colore di ciascun vettore
void LittleObject::printOnfile(char color, char c, ofstream& output)
{
	vector<Vec3b>* V;
	vector<Vec3b>::iterator it;
	V=selectVector(color);
	for(it=V->begin(); it<V->end(); it++)
	{
		output << " " << (int)((*it)[0]) << " " << (int)((*it)[1]) << " " << (int)((*it)[2]) << " " << c; 
	}

}

//metodo per visualizzare l'immagine
char LittleObject::showImage()
{
	if(this->mode_r) 
	{
		ros::spinOnce();
	}
	if(!(this->img.empty()))
	{
		imshow("Little Endian Interface",this->img);
	}
	if(this->mode_r)
	{
		char c = waitKey(1);
		if ( c != -1)
		    return c;
		else
		    return 0;
	}
	else return waitKey(0);
	
}

//metodo per azzerare i vettori dei colori
void LittleObject::cleanVectors()
{
	this->r.clear();
	this->g.clear();
	this->b.clear();
}

