#ifndef PIXEL_MAP_H
#define PIXEL_MAP_H

#include <iostream>
#include <vector>
#include <map>
#include <set>

#include "Blob.h"
#include "ColorClassifier.h"

#define   DEFAULT_BALL_RADIUS 0.22

using namespace std;

class PixelMap
{
 public:

 // Costruttore di Default
 // PixelMap (ColorClassifier*,unsigned char*);
 PixelMap();
 PixelMap (unsigned char*,int,int,ColorClassifier*, int ppb=3); 
 ~PixelMap();

 map < char, map< int,Blob* > >* GetBlobs() {return &mBlobs;};
 map <int, Blob*>* GetBlobs(char c) {return &mBlobs[c]; };

 void ClearBlobColorList();
 set<char> GetBlobColorList();

 // Blob growing is done in a rectangular window or in the whole image with step one (default)
 void BlobGrowing(int step=1, int StartWindowX=-1, int StartWindowY=130, int EndWindowX=-1, int EndWindowY=-1);
														
 // Save find Blob to file
 int SaveBlobsToFile(char* filename,int filter=0);

 // Reference to the classified image
 unsigned char* mColorImage;

 private:
 // Reference to the image
 unsigned char* mImage;

 // Width of the image
 int mWidth; 
 // Height of the image
 int mHeight;
 // Vectors used during BlobGrowing
 vector<int> curr_line, prev_line;
 // Number of byte per pixel
 int Pixel4Byte;
 // The Color Classifier
 ColorClassifier* mColor;
 // Variables used to hold temporary classifications
 char PxLabel0,PxLabel1,PxLabel2,PxLabel3,PxLabel4;
 // Variable used to hold the Blob list
 map < char, map< int,Blob* > > mBlobs;
  
 int nextBlobId;

 // This function in used to selec valid colors for blob growing
 bool ValidPixel(char);
 set<char> mBlobColorList;

};

inline bool PixelMap::ValidPixel(char P)
{
  if(mBlobColorList.empty())
    return P!='U'; 
  else
    return (bool)mBlobColorList.count(P);
}

#endif

