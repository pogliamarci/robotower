// This is the class to implement a color dataset for the learning algorithm

#ifndef COLOR_SAMPLE_H
#define COLOR_SAMPLE_H

#include <iterator>
#include <iostream>
#include <set>

using namespace std;

typedef enum {RGB24,YUV411} ColorSpace;

/**This is a class that containes the information about the choosen samples fork
 *for the learning algoritm.
 */
class ColorSample{
  /** This are the Color Coordinates
   */
  unsigned char R, G, B;
  unsigned char Y, U, V;
  /**	This contains the label assigned to the pixel
   */
  unsigned char Class;   
  unsigned int SamplingId;
 public:
  ColorSample():R(0),G(0),B(0),Y(0),U(0),V(0),Class(0),SamplingId(0){}
  ColorSample(unsigned char RR, unsigned char GG, unsigned char BB, unsigned char CC, unsigned int SI, ColorSpace CS=RGB24);

  unsigned char getR() const { return R; }
  unsigned char getG() const { return G; }
  unsigned char getB() const { return B; }
  unsigned char getY() const { return Y; }
  unsigned char getU() const { return U; }
  unsigned char getV() const { return V; }

  unsigned char getClass() const { return Class; }
  void setClass(unsigned char CL) {Class = CL;}
  unsigned int getSamplingId() const { return SamplingId; }
  void setSamplingId(unsigned int SI) { SamplingId = SI; }

  
  static int yuv2rgb(unsigned char y, unsigned char u, unsigned char v);
  static int rgb2yuv(unsigned char r, unsigned char g, unsigned char b);
};


bool operator==(const ColorSample&, const ColorSample&);
int operator<(const ColorSample&, const ColorSample&);
ostream& operator<<(ostream&, const ColorSample&);

/** This is the class to implement a color dataset for the learning algorithm.
 */
class ColorDataset{
  set<ColorSample> Samples;
 public:
  void insert(ColorSample);
  void undo(unsigned int);
  void deleteClass(unsigned char);
  void clear();
  unsigned int size();
  set<ColorSample>::iterator begin() /*const*/ { return Samples.begin(); };
  set<ColorSample>::iterator end() /*const*/ { return Samples.end(); };
  void print(ostream&);
  void save(char* filename);
  void load(const char* filename);
};

#endif
