/* Receptors : This part of the project develops the class Receptor and his son classes : Cross , XCross  and Square */

#ifndef RECEPTOR_H
#define RECEPTOR_H

#include <stdlib.h>

typedef enum {NO_FILTER, SQUARE, CROSS, XCROSS, HYSTERESIS} ReceptorType;

 /** A receptor is a struct that makes an average value of the nearby pixels 
  *  (it works more or less like a filter) 
  * 
  *  Four different types of Receptor : Cross, X Cross , Square , NoFilter.
  *
  *
  */
class Receptor
{
 protected:
  /**This are the coordinates on the image of the center of the receptor.
   */
  int PosX,PosY; 
  /**This contains the label corresponding to the classified color .
   *(made with the DBScan or the Knn Algorithm).
   */
  unsigned char label; 
  /**Killed is a boolean to explain if the receptor must be used.
   */
  bool killed;
  /**Radial distance in the scene 
   */
  float rho; 
  /**Angular distance in the scene 
   */
  float theta;
  /**ColorValue stores the RGB Values of the receptor.
   */
  int ColorValue[3];

  /** This is the type of filtering applyed to the receptor
   */
  ReceptorType type;

  /** The BlobID is used for rewriting during in blob-growing procedure 
   */
  int BlobID;

  /** The Connected receptors are used during blob growing and feature extraction
   *  to speed up the computation. They are computed by the ReceptorMap.
   */
  Receptor* Connected_1;
  Receptor* Connected_2;
  Receptor* Connected_3;
  Receptor* Connected_4;
  Receptor* Connected_5; // the next anglewise
  Receptor* Connected_6; // the next radiuswise
  
  float distance;

 public:
     
  Receptor():PosX(0),PosY(0),label('U'),killed(false),rho(-1),theta(-1),type(NO_FILTER),
    Connected_1(NULL),Connected_2(NULL),Connected_3(NULL),Connected_4(NULL),Connected_5(NULL),Connected_6(NULL){};
  Receptor(int ,int ,unsigned char* ,int, bool, ReceptorType);
		
  int GetPosX(){return PosX;} 
  int GetPosY(){return PosY;}	
  int GetValueR(){return ColorValue[0];}
  int GetValueG(){return ColorValue[1];}
  int GetValueB(){return ColorValue[2];}

  void SetLabel(unsigned char lab){label = lab;}
  unsigned char GetLabel(){return label;}

  void SetKilled(bool killedstate){killed = killedstate;}
  bool GetKilled(){return killed;}

  float GetRho(){return rho;}
  void  SetRho(float r){rho = r;}
  
  float GetTheta(){return theta;}
  void  SetTheta(float t){theta = t;}
  
  int GetBlobID(){return BlobID;}
  void SetBlobID(int BID){BlobID=BID;};

  Receptor* GetConnected(int selector);
  void SetConnected(int selector, Receptor* R);
  
  ReceptorType GetRType(){return type;};
  
  /**The function GetRGB is implemented in a different way in every receptor type 
   *(for the computation of the average value).
   */
  void GetRGB(unsigned char* ,int);

  /**The function GetRGB is implemented in a different way in every receptor type 
   *(for the computation of the average value).
   */
  void GetYUV(unsigned char* ,int);
		
  //virtual void Hyst_Classify(guint8*,int, Classifier*) {}
};

inline Receptor* Receptor::GetConnected(int selector)
{
  switch(selector)
    {
    case 1: return Connected_1; break;
    case 2: return Connected_2; break;
    case 3: return Connected_3; break;
    case 4: return Connected_4; break;
    case 5: return Connected_5; break;
    case 6: return Connected_6; break;
    default: return NULL;
    }
}

inline void Receptor::SetConnected(int selector, Receptor* R)
{
  switch(selector)
    {
    case 1: Connected_1 = R; break;
    case 2: Connected_2 = R; break;
    case 3: Connected_3 = R; break;
    case 4: Connected_4 = R; break;
    case 5: Connected_5 = R; break;
    case 6: Connected_6 = R; break;
    }
}
  
#endif
