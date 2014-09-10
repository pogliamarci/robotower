// This is the implementation of ColorDataset Methods

#include "ColorDataset.h"
#include <fstream>
#include <iostream>

using namespace std;

#define COLOR_COMPRESSION_FACTOR 4;

ColorSample::ColorSample(unsigned char RR, unsigned char GG, unsigned char BB, unsigned char CC, unsigned int SI, ColorSpace CS)
{
  unsigned int pixel32;
  unsigned char *pixel = (unsigned char *)&pixel32;

  if(CS==YUV411){
    Y=RR;
    U=GG;
    V=BB;
    Class=CC;
    SamplingId = SI;
    
    pixel32 = yuv2rgb(R, G, B);
    R = pixel[0];
    G = pixel[1];
    B = pixel[2];
  }else{
    R=RR;
    G=GG;
    B=BB;
    Class=CC;
    SamplingId = SI;
    pixel32 = rgb2yuv(R, G, B);
    Y = pixel[0];
    U = pixel[1];
    V = pixel[2];
  }
}

int ColorSample::yuv2rgb(unsigned char y, unsigned char u, unsigned char v)
{
  unsigned int pixel32;
  unsigned char *pixel = (unsigned char *)&pixel32;
  int r, g, b;
  int C, D, E; 
  
  C = y - 16;
  D = u - 128;
  E = v - 128;
  r = ( 298 * C           + 409 * E + 128 ) >> 8;
  g = ( 298 * C - 100 * D - 208 * E + 128 ) >> 8;
  b = ( 298 * C + 516 * D           + 128 ) >> 8;
  
  // Even with proper conversion, some values still need clipping.
  if (r > 255) r = 255;
  if (g > 255) g = 255;
  if (b > 255) b = 255;
  if (r < 0) r = 0;
  if (g < 0) g = 0;
  if (b < 0) b = 0;
  
  pixel[0] = r;
  pixel[1] = g;
  pixel[2] = b;
  pixel[3] = 0;
  
  return pixel32;
}

int ColorSample::rgb2yuv(unsigned char r, unsigned char g, unsigned char b)
{
  unsigned int pixel32;
  unsigned char *pixel = (unsigned char *)&pixel32;
  int y, u, v;
  
  y = ( (  66 * r + 129 * g +  25 * b + 128) >> 8) +  16;
  u = ( ( -38 * r -  74 * g + 112 * b + 128) >> 8) + 128;
  v = ( ( 112 * r -  94 * g -  18 * b + 128) >> 8) + 128;
  
  // Even with proper conversion, some values still need clipping.
  if (y > 235) y = 235;
  if (u > 255) u = 255;
  if (v > 255) v = 255;
  if (y < 16) y = 16;
  if (u < 0) u = 0;
  if (v < 0) v = 0;
  
  pixel[0] = y;
  pixel[1] = u;
  pixel[2] = v;
  pixel[3] = 0;
  
  return pixel32;
}


bool
operator==(const ColorSample& first, const ColorSample& second){
  return (first.getR() == second.getR() && first.getG() == second.getG() &&
	  first.getB() == second.getB() && first.getClass() == second.getClass());
}

int
operator<(const ColorSample& first, const ColorSample& second){
  if(first.getR() == second.getR())
    if(first.getG() == second.getG())
      if(first.getB() == second.getB())
	if(first.getClass() == second.getClass())
	  return first.getSamplingId() < second.getSamplingId();
	else
	  return first.getClass() < second.getClass();
      else
	return first.getB() < second.getB();
    else
      return first.getG() < second.getG();
  else 
    return first.getR() < second.getR();
}

ostream&
operator<<(ostream& os, const ColorSample& s) {
  return os << "[" << (unsigned int)s.getR() << "," << (unsigned int)s.getG() << "," << (unsigned int)s.getB() << "] -> " << s.getClass() << " | " << s.getSamplingId();
}

void
ColorDataset::insert(ColorSample CS)
{
  Samples.insert(CS);
}

void
ColorDataset::undo(unsigned int SamplingId)
{
  set<ColorSample>::iterator itr;
  for(itr = Samples.begin(); itr != Samples.end(); itr++)
    if(itr->getSamplingId() == SamplingId)
      Samples.erase(itr);
}

void
ColorDataset::deleteClass(unsigned char C)
{
  set<ColorSample>::iterator itr;
  for(itr = Samples.begin(); itr != Samples.end(); itr++)
    if(itr->getClass() == C)
      Samples.erase(itr);
}

void
ColorDataset::clear()
{
  Samples.clear();
}

unsigned int
ColorDataset::size()
{
  return Samples.size();
}

void 
ColorDataset::print(ostream& os)
{
  copy(Samples.begin(), Samples.end(),ostream_iterator<ColorSample>(os, "\n"));
}

void ColorDataset::save(char* filename)
{
  ofstream File;
  File.open(filename,ios::out);
  File << this->size()<<' ';
  set<ColorSample>::iterator SamplesIterator;
  for(SamplesIterator = this->begin(); SamplesIterator != this->end(); SamplesIterator++)
    {
      File << (int)SamplesIterator->getR() << ' ';
      File << (int)SamplesIterator->getG() << ' ';
      File << (int)SamplesIterator->getB() << ' ';     
      File << SamplesIterator->getClass() << ' ';
    }
  File.close();
    	 
}
void ColorDataset::load(const char* filename)
{
  unsigned int size;//temp;
  unsigned char  Class;
  int R, G, B;
  ColorSample ActualColorSample;
   
 
  ifstream File;
  File.open(filename,ios::in);
  File >> size;
  for ( int i =0; i< (int)size; i++)
    {
      File >> R;
      File >> G;
      File >> B;
      File >> Class;
      
      ActualColorSample=ColorSample((unsigned char)R,(unsigned char) G,(unsigned char) B, Class,0);
      this->insert(ActualColorSample);	
    }	
  File.close();   	 
}
