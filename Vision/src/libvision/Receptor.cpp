#include "Receptor.h"
using namespace std;

Receptor::Receptor(int x, int y, unsigned char* pixelpoint ,int rowstride, bool killed_state, ReceptorType myType)
{
  PosX=x;
  PosY=y;
  killed=killed_state;
  type=myType;
  Connected_1 = NULL;
  Connected_2 = NULL;
  Connected_3 = NULL;
  Connected_4 = NULL;
  Connected_5 = NULL;
  Connected_6 = NULL;
  this->GetRGB(pixelpoint,rowstride);
};

void Receptor::GetRGB(unsigned char* pixelpoint, int rowstride)
{
  unsigned char* pixel;// = NULL;	
  int R,G,B;
  R=G=B=0;
  
  if(killed)
    {
      ColorValue[0]=0;
      ColorValue[1]=0;
      ColorValue[2]=0;
      return;
    }
  
  switch(type){
  case CROSS:	
    for (int k=-1;k<2;k++)
      for (int h=-1;h<2;h++)
	if (abs(h)==abs(k))
	  continue;
	else
	  {
	    pixel =  pixelpoint + rowstride*(PosY+k) + (PosX+h)*3;
	    R = R + *pixel;
	    G = G + *(pixel+1);
	    B = B + *(pixel+2);
	  }
    R = R >> 3;//8; //shift 3
    G = G >> 3;
    B = B >>3; //8;
    pixel =  pixelpoint + rowstride*(PosY) + (PosX)*3;
    ColorValue[0] =  R + ((*pixel)>>1);	//shift 1
    ColorValue[1] =  G + (*(pixel+1) >> 1);
    ColorValue[2] =  B + (*(pixel+2) >> 1);
    break;
  case XCROSS:
    for (int k=-1;k<2;k++)
      for (int h=-1;h<2;h++)
	if (abs(h)+abs(k)>1)
	  {
	    pixel =  pixelpoint + rowstride*(PosY+k) + (PosX+h)*3;
	    R = R + *pixel;
	    G = G + *(pixel+1);
	    B = B + *(pixel+2);
	  }	
    R = R >> 3; //shift 3
    G = G >> 3;
    B = B >> 3;
    pixel =  pixelpoint + rowstride*(PosY) + (PosX)*3;
    ColorValue[0] =  R + (*pixel >> 1);	//shift 1
    ColorValue[1] =  G + (*(pixel+1) >> 1);
    ColorValue[2] =  B + (*(pixel+2) >> 1);
    break;
  case SQUARE:
    for (int k=-1;k<2;k++)
      for (int h=-1;h<2;h++)
	if ((h==0)&&(k==0))
	  continue;
	else
	  {
	    pixel =  pixelpoint + rowstride*(PosY+k) + (PosX+h)*3;
	    R = R + *pixel;
	    G = G + *(pixel+1);
	    B = B + *(pixel+2);
	  }
    R = R >> 4; //shift 4
    G = G >> 4;
    B = B >> 4;
    pixel =  pixelpoint + rowstride*(PosY) + (PosX)*3;
    ColorValue[0] =  R + (*pixel >> 1);	//shift 1
    ColorValue[1] =  G + (*(pixel+1) >> 1);
    ColorValue[2] =  B + (*(pixel+2) >> 1);
    break;
  case NO_FILTER:
  case HYSTERESIS:
    pixel =  pixelpoint + rowstride*(PosY) + (PosX)*3;
    ColorValue[0] = *pixel;
    ColorValue[1] = *(pixel+1);
    ColorValue[2] = *(pixel+2);
    break;
  default:
    break;
  }
};


void Receptor::GetYUV(unsigned char* pixelpoint, int rowstride)
{
    unsigned char* pixel;// = NULL;	
	
  int Y,U,V;
  Y=U=V=0;
  
  if(killed)
    {
      ColorValue[0]=0;
      ColorValue[1]=0;
      ColorValue[2]=0;
      return;
    }
  
  switch(type){
  case CROSS:	
	  for (int k=-1;k<2;k++)
		  for (int h=-1;h<2;h++)
			  if (abs(h)==abs(k))
				  continue;
			  else
			  {
				  pixel = pixelpoint+(PosY+k)*((rowstride<<2)>>3) + ((PosX+h)>>2)*6;
				  Y = Y + *(pixel + (int)(((PosX+h) & 0x2)>>1) + (int)((PosX+h) & 0x3) + 1 );
				  U = U + *(pixel);
				  V = V + *(pixel + 3);
			  }
			  
			  Y = Y >> 3; //8; //shift 3
			  U = U >> 3;
			  V = V >> 3; //8;
			  pixel = pixelpoint+(PosY)*((rowstride<<2)>>3) + ((PosX)>>2)*6;
			  ColorValue[0] =  Y + (*(pixel + (int)((PosX & 0x2)>>1) + (int)(PosX & 0x3) + 1 )>>1);	//shift 1
			  ColorValue[1] =  U + (*(pixel)>> 1);
			  ColorValue[2] =  V + (*(pixel+3) >> 1);
			  break;
  case XCROSS:
	  for (int k=-1;k<2;k++)
		  for (int h=-1;h<2;h++)
			  if (abs(h)+abs(k)>1)
			  {
				  pixel = pixelpoint+(PosY+k)*((rowstride<<2)>>3) + ((PosX+h)>>2)*6;
				  Y = Y + *(pixel + (int)((PosX & 0x2)>>1) + (int)(PosX & 0x3) + 1 );
				  U = U + *(pixel);
				  V = V + *(pixel + 3);
			  }	
			  Y = Y >> 3; //8; //shift 3
			  U = U >> 3;
			  V = V >> 3; //8;
			  pixel = pixelpoint+(PosY)*((rowstride<<2)>>3) + ((PosX)>>2)*6;
			  ColorValue[0] =  Y + (*(pixel + (int)((PosX & 0x2)>>1) + (int)(PosX & 0x3) + 1 )>>1);	//shift 1
			  ColorValue[1] =  U + (*(pixel)>> 1);
			  ColorValue[2] =  V + (*(pixel + 3) >> 1);
			  break;
  case SQUARE:
	  for (int k=-1;k<2;k++)
		  for (int h=-1;h<2;h++)
			  if ((h==0)&&(k==0))
				  continue;
			  else
			  {
				  pixel = pixelpoint+(PosY+k)*((rowstride<<2)>>3) + ((PosX+h)>>2)*6;
				  Y = Y + *(pixel + (int)((PosX & 0x2)>>1) + (int)(PosX & 0x3) + 1 );
				  U = U + *(pixel);
				  V = V + *(pixel + 3);
			  }
			  Y = Y >> 4; //shift 4
			  U = U >> 4;
			  V = V >> 4;
			  pixel = pixelpoint+(PosY)*((rowstride<<2)>>3) + ((PosX)>>2)*6;
			  ColorValue[0] =  Y + (*(pixel + (int)((PosX & 0x2)>>1) + (int)(PosX & 0x3) + 1 )>>1);	//shift 1
			  ColorValue[1] =  U + (*(pixel)>> 1);
			  ColorValue[2] =  V + (*(pixel + 3) >> 1);
			  break;
  case NO_FILTER:
  case HYSTERESIS:
			  pixel = pixelpoint+(PosY)*((rowstride<<2)>>3) + ((PosX)>>2)*6;
			  ColorValue[0] =  *(pixel + (int)((PosX & 0x2)>>1) + (int)(PosX & 0x3) + 1 );
			  ColorValue[1] =  *(pixel);
			  ColorValue[2] =  *(pixel + 3);
			  break;
  default:
    break;
  }
};

/*
  void HysteresisR::Hyst_Classify( guint8* pixelpoint , int rowstride , Classifier* Db )
  {
  if(killed==true)
  return;	
	
  map<unsigned char,unsigned int> Counters;
  map<unsigned char,unsigned int>::iterator CountItr;
  unsigned char MaxClass = 'U';
  unsigned int MaxCount = 0;	

  Counters['R']=0;  Counters['B']=0;  Counters['Y']=0;
  Counters['G']=0;  Counters['K']=0;  Counters['W']=0;
  Counters['M']=0;  Counters['C']=0;  Counters['U']=0;
  MaxClass='U'; MaxCount=0;
	
  guint8* pixel = NULL;	
  unsigned char PixelLabel; 	

  for (int k=-1;k<2;k++)
  {
  for (int h=-1;h<2;h++)
  {	
  pixel =  pixelpoint + rowstride*(PosY+k) + (PosX+h)*3;
  PixelLabel = Db->get_color( *pixel , *(pixel+1) , *(pixel+2));
  Counters[ PixelLabel ] ++;
  }
  }
  
  for(CountItr = Counters.begin(); CountItr != Counters.end(); CountItr++)
  {
  if(CountItr->second > MaxCount)
  {
  MaxCount = CountItr->second;
  MaxClass = CountItr->first; 
  }	  
  }
  label= MaxClass;
     
  return;		
  };
*/
