#include "Blob.h"

int operator<(const Blob &first, const Blob &second)
{ 
  return first.GetBlobID() < second.GetBlobID();
}

int operator<(const Equivalence &first, const Equivalence &second)
{ 
  if (first.GetFirst() != second.GetFirst())
    return first.GetFirst() < second.GetFirst();
  else if (first.GetSecond() != second.GetSecond())
    return first.GetSecond() < second.GetSecond();
  else
    return first.GetClass() < second.GetClass();
}

void Blob::merge(int i, int j)
{
  NumPix++;

  // This has to be removed :)
  if(MinX > j) MinX=j;
  if(MaxX < j) MaxX=j;
  if(MinY > i) MinY=i;
  if(MaxY < i) MaxY=i;
};

void Blob::merge(Receptor* R)
{
  NumRec++;

  // This has to be removed :)
  if(MinX > R->GetPosX()) MinX=R->GetPosX();
  if(MaxX < R->GetPosX()) MaxX=R->GetPosX();
  if(MinY > R->GetPosY()) MinY=R->GetPosY();
  if(MaxY < R->GetPosY()) MaxY=R->GetPosY();

  // Merging of rho information
  if(MinRho   > R->GetRho()) MinRho=R->GetRho();
  if(MaxRho   < R->GetRho()) MaxRho=R->GetRho();

  // This mess is to manage the cylindric map
  if(MinTheta > MaxTheta)  // This is a Blob around 0
    {
      if(fabs(MinTheta - R->GetTheta()) < fabs(MaxTheta - R->GetTheta())) // Closer to the minimum
	{
	  if(MinTheta > R->GetTheta()) MinTheta=R->GetTheta();
	}
      else // Closer to the maximum
	{
	  if(MaxTheta < R->GetTheta()) MaxTheta=R->GetTheta();
	}
    }
  else // This is a "normal blob"
    {
      if((MaxTheta - R->GetTheta()) > POSITIVE_BIG_ANGLE_THRESHOLD ||
	 (MinTheta - R->GetTheta()) > POSITIVE_BIG_ANGLE_THRESHOLD ) // Blob "below 0" Receptor "above 0" 
	{
	  MaxTheta = R->GetTheta();
	}
      else if((MaxTheta - R->GetTheta()) < NEGATIVE_BIG_ANGLE_THRESHOLD ||
	      (MinTheta - R->GetTheta()) < NEGATIVE_BIG_ANGLE_THRESHOLD ) // Blob "above 0" Recptor "below 0" 
	{
	  MinTheta = R->GetTheta();
	}
      else // Plain case :)
	{
	  if(MinTheta > R->GetTheta()) MinTheta=R->GetTheta();
	  if(MaxTheta < R->GetTheta()) MaxTheta=R->GetTheta();
	}
    }

};

void Blob::merge(Blob* B)
{
  // Just merge valid blobs
  if(Valid && B->GetValid())
    {
      B->SetValid(false);
      
      NumRec+=B->GetNumRec();
      NumPix+=B->GetNumPix();
      
      // This will be removed
      if(MinX > B->GetMinX()) MinX=B->GetMinX();
      if(MaxX < B->GetMaxX()) MaxX=B->GetMaxX();
      if(MinY > B->GetMinY()) MinY=B->GetMinY();
      if(MaxY < B->GetMaxY()) MaxY=B->GetMaxY();

      if(type == RECEPTOR_BLOB)
      {
         // Start merging rho information 
         if(MinRho > B->GetMinRho()) MinRho=B->GetMinRho();
         if(MaxRho < B->GetMaxRho()) MaxRho=B->GetMaxRho();
      
         // Thi mess is to manage the cylindric map
         if((MinTheta > MaxTheta) && (B->GetMinTheta() > B->GetMaxTheta())) // Both blobs around 0
	 {
	     if(MinTheta > B->GetMinTheta()) MinTheta=B->GetMinTheta();
	     if(MaxTheta < B->GetMaxTheta()) MaxTheta=B->GetMaxTheta();
 	 }
         else if (MinTheta > MaxTheta) // Only this blob around 0
	 {
	     if((MinTheta - B->GetMinTheta()) > POSITIVE_BIG_ANGLE_THRESHOLD) // Blob closer to the maximum
	     {
	        if(MaxTheta < B->GetMaxTheta()) MaxTheta=B->GetMaxTheta();
	     }
	     else if((MaxTheta - B->GetMaxTheta()) < NEGATIVE_BIG_ANGLE_THRESHOLD) // Blob closer to the minimum
	     {
	        if(MinTheta > B->GetMinTheta()) MinTheta=B->GetMinTheta();
	     }
	     else
	     {
	        if(MinTheta > B->GetMinTheta()) MinTheta=B->GetMinTheta();
	        if(MaxTheta < B->GetMaxTheta()) MaxTheta=B->GetMaxTheta();
	     }	  
	 }
         else if (B->GetMinTheta() > B->GetMaxTheta()) // Only the other blob around 0
	 {
	    if((MinTheta - B->GetMinTheta()) < NEGATIVE_BIG_ANGLE_THRESHOLD) // This is closer to the maximum
	    {
	      if(MinTheta < B->GetMinTheta()) MinTheta=B->GetMinTheta();
	      if(MaxTheta < B->GetMaxTheta()) MaxTheta=B->GetMaxTheta();
	    }
	    else if((MaxTheta - B->GetMaxTheta()) > POSITIVE_BIG_ANGLE_THRESHOLD) // This is closer to the minimum
	    {
	      if(MinTheta > B->GetMinTheta()) MinTheta=B->GetMinTheta();
	      if(MaxTheta > B->GetMaxTheta()) MaxTheta=B->GetMaxTheta();
	    }
	    else
	    {
	      if(MinTheta > B->GetMinTheta()) MinTheta=B->GetMinTheta();
	      if(MaxTheta < B->GetMaxTheta()) MaxTheta=B->GetMaxTheta();
	    }	  
	  }
          else // No blob around 0
	  {
	     if((MaxTheta - B->GetMaxTheta()) > POSITIVE_BIG_ANGLE_THRESHOLD ||
	        (MinTheta - B->GetMinTheta()) > POSITIVE_BIG_ANGLE_THRESHOLD ) // This "below 0", B "above 0" 
	     {
	        MaxTheta = B->GetMaxTheta();
	     }
	     else if((MaxTheta - B->GetMaxTheta()) < NEGATIVE_BIG_ANGLE_THRESHOLD ||
		     (MinTheta - B->GetMaxTheta()) < NEGATIVE_BIG_ANGLE_THRESHOLD ) // Blob "above 0" Recptor "below 0" 
	     {
	         MinTheta = B->GetMinTheta();
	     }
	     else // Plain case :)
	    {
	      if(MinTheta > B->GetMinTheta()) MinTheta=B->GetMinTheta();
	      if(MaxTheta < B->GetMaxTheta()) MaxTheta=B->GetMaxTheta();
	    }
	}
      
      FirstCrown = FirstCrown || B->GetFirstCrown();
      }
    }
};

