#include "PixelMap.h"

PixelMap::PixelMap()
{
  mImage=NULL;
  mColorImage=NULL;
  mWidth=0; 
  mHeight=0;
  curr_line.clear();
  prev_line.clear();
  Pixel4Byte=3;
  mColor=NULL;
  PxLabel0=PxLabel1=PxLabel2=PxLabel3=PxLabel4='U';
  mBlobs.clear();
  nextBlobId=1;
  mBlobColorList.clear();
  //BallRadius = (float)DEFAULT_BALL_RADIUS;
  //Undistort = KOVESI;
  //BallX = BallY = BallZ = 0.0;
}

PixelMap::PixelMap(unsigned char* img ,int width, int height, ColorClassifier* cc, int ppb)
{
  mImage=img;
  mColorImage = new unsigned char[width*height];
  mWidth=width; 
  mHeight=height;
  curr_line.reserve(width);
  prev_line.reserve(width);
  Pixel4Byte=ppb;
  mColor=cc;
  PxLabel0=PxLabel1=PxLabel2=PxLabel3=PxLabel4='U';
  mBlobs.clear();
  nextBlobId=1;
  mBlobColorList.clear();
//  BallRadius = BallRadius;
//  Undistort = undist;
//  BallX = BallY = BallZ = 0.0;
}

PixelMap::~PixelMap() {
	delete[] mColorImage;
	/* free all the dynamically allocated memory in the map */
	/* WARNING: DON'T USE ANY POINTER RETURNED FROM THE PIXELMAP
	 * OBJECT AFTER CALLING THE DESTRUCTOR! (e.g. the Blobs...)
	 */
	map < char, map< int,Blob* > >::iterator it;
	for ( it=mBlobs.begin() ; it != mBlobs.end(); it++ ) {
		map< int,Blob* >::iterator internalItr;
		for(internalItr = (*it).second.begin();
				internalItr != (*it).second.end(); internalItr++) {
			delete (*internalItr).second;
		}
	}
	mBlobs.clear();
}

void PixelMap::BlobGrowing(int step, int StartWindowX, int StartWindowY, int EndWindowX, int EndWindowY)
{  

//cout << "----> BlobGrowing    [ok]" << endl;

/* int time = 0; */

map < pair <int,int>, char > EquivSet;
map < pair <int,int>, char > EquivSwap;

map < pair <int,int>, char >::iterator EquivItr;
map < pair <int,int>, char >::iterator SwapItr;
map < pair <int,int>, char >::iterator NewItr;

vector < Equivalence > NewEquiv;
vector < Equivalence >::iterator NewEquivItr1;
vector < Equivalence >::iterator NewEquivItr2;
vector < Equivalence >::iterator NewEquivItr;

	
int rowstride = mWidth*Pixel4Byte;  

	  
map < char, map< int,Blob* > >::iterator mBlobsItr;

	
map < int,Blob* >::iterator BlobItr;

	
for(mBlobsItr = mBlobs.begin(); mBlobsItr != mBlobs.end(); mBlobsItr++)   
{
	for(BlobItr = mBlobsItr->second.begin(); BlobItr != mBlobsItr->second.end(); BlobItr++)
	{
		delete BlobItr->second;
	}

   	mBlobsItr->second.clear();
}

mBlobs.clear();

nextBlobId=1;
  
for(unsigned int k=0 ; k<prev_line.size() ;k++)
{
    prev_line[k]=0; //vector
}

for(unsigned int k=0 ; k<curr_line.size() ;k++) 
{
   curr_line[k]=0;  //vector

}



// This is to classify the whole image :)
if(StartWindowX == -1) StartWindowX=0;
if(StartWindowY == -1) StartWindowY=0;
if(EndWindowX == -1) EndWindowX=mWidth;
if(EndWindowY == -1) EndWindowY=mHeight;

if(StartWindowX > EndWindowX)
{
      int swap;
      swap = StartWindowX;
      StartWindowX = EndWindowX;
      EndWindowX = swap;
}

if(StartWindowY > EndWindowY)
{
      int swap;
      swap = StartWindowY;
      StartWindowY = EndWindowY;
      EndWindowY = swap;
}


/* time = getTime(); */

	


for(int i=StartWindowY; i < EndWindowY; i+=step)
{
	for(int j=StartWindowX; j < EndWindowX; j+=step)
    	{
		mColorImage[mWidth*i+j]=mColor->get_color((unsigned int)*((mImage + i*rowstride + j*Pixel4Byte)+0), 
							  (unsigned int)*((mImage + i*rowstride + j*Pixel4Byte)+1),
							  (unsigned int)*((mImage + i*rowstride + j*Pixel4Byte)+2));
	}
}



//printf("\tTiming:: To do Classification \t= %d\n", getTime()-time);     

// Begin the blob growing code

/* time = getTime();*/

  for(int i=StartWindowY; i<EndWindowY;i=i+step)	
    {
      for(int j=StartWindowX; j<EndWindowX;j=j+step)
	{
	 if(i==StartWindowY && j==StartWindowX) //################# First Pixel
	  {
	    PxLabel0 = (char)mColorImage[mWidth * i + j];
	    if(ValidPixel(PxLabel0))
	      {
		mBlobs[PxLabel0][nextBlobId] = new Blob(nextBlobId,i,j);
		curr_line[j]=nextBlobId++;
	      }
	  }
	  else if (i==StartWindowY && j!=StartWindowX) //########################### First Row
	    {
	      PxLabel0 = (char)mColorImage[mWidth*i + j];
	      if(ValidPixel(PxLabel0))
		{ 
		  PxLabel1=(char)mColorImage[mWidth*i + (j-step)];
		  if(PxLabel0==PxLabel1){
		    curr_line[j]=curr_line[j-step];
		    mBlobs[PxLabel0][curr_line[j]]->merge(i,j);
		  }
		  else{
		    mBlobs[PxLabel0][nextBlobId] = new Blob(nextBlobId,i,j);
		    curr_line[j]=nextBlobId++;
		  }
		}
	    }
	  else if (i!=StartWindowY && j==StartWindowX) //################# First Column
	    {
	      PxLabel0 = (char)mColorImage[mWidth*i + j];
	      if(ValidPixel(PxLabel0))
		{   
		  PxLabel3=(char)mColorImage[mWidth*(i-step) + j]; 
		  PxLabel4=(char)mColorImage[mWidth*(i-step) + (j+step)];
		  if(PxLabel0==PxLabel3){
		    if((curr_line[j]!=0) && (prev_line[j]!=0) && (curr_line[j]!=prev_line[j]))
		      {
			if(curr_line[j]<prev_line[j]){
			  pair<int,int> p(curr_line[j],prev_line[j]);
			  EquivSet[p]=PxLabel0;
			}else{ 
			  pair<int,int> p(prev_line[j],curr_line[j]);
			  EquivSet[p]=PxLabel0;
			}   
			//Equivalence E(curr_line[j],prev_line[j],PxLabel0);
			//EquivSet.insert(E);
		      }
		    else if(curr_line[j]==0 && prev_line[j]!=0){
		      curr_line[j]=prev_line[j];
		      mBlobs[PxLabel0][curr_line[j]]->merge(i,j);
		    }
		  }
		  if(PxLabel0==PxLabel4){
		    if((curr_line[j]!=0) && (prev_line[j+1]!=0) && (curr_line[j]!=prev_line[j+1])) 
		      {
			if(curr_line[j]<prev_line[j+step]){
			  pair<int,int> p(curr_line[j],prev_line[j+step]);
			  EquivSet[p]=PxLabel0;
			}else{
			  pair<int,int> p(prev_line[j+step],curr_line[j]);
			  EquivSet[p]=PxLabel0;
			}
			//Equivalence E(curr_line[j],prev_line[j+1],PxLabel0);
			//EquivSet.insert(E);
		      }
		    else if(curr_line[j]==0 && prev_line[j+step]!=0){
		      curr_line[j]=prev_line[j+step];
		      mBlobs[PxLabel0][curr_line[j]]->merge(i,j);
		    }
		  }
		  if(PxLabel0 != PxLabel3 && PxLabel0 != PxLabel4){
			mBlobs[PxLabel0][nextBlobId] = new Blob(nextBlobId,i,j);
		    curr_line[j]=nextBlobId++;
		  }
		}
	    }
	  else if (j==EndWindowX-step) //################################# Last Column
	    {
	      PxLabel0 = (char)mColorImage[mWidth*i + j];
	      if(ValidPixel(PxLabel0))
		{   
		  PxLabel1=(char)mColorImage[mWidth*i + (j-step)];
		  PxLabel2=(char)mColorImage[mWidth*(i-step) + (j-step)];
		  PxLabel3=(char)mColorImage[mWidth*(i-step) + j];
		  if(PxLabel0==PxLabel1){
		    if((curr_line[j]!=0) && (curr_line[j-step]!=0) && (curr_line[j]!=curr_line[j-step]))
		      {
			if(curr_line[j]<curr_line[j-step]){
			  pair<int,int> p(curr_line[j],curr_line[j-step]);
			  EquivSet[p]=PxLabel0;
			}else{
			  pair<int,int> p(curr_line[j-step],curr_line[j]);
			  EquivSet[p]=PxLabel0;
			}
			//Equivalence E(curr_line[j],curr_line[j-1],PxLabel0);
			//EquivSet.insert(E);
		      }
		    else if(curr_line[j]==0 && curr_line[j-step]!=0){
		      curr_line[j]=curr_line[j-step];
		      mBlobs[PxLabel0][curr_line[j]]->merge(i,j);
		    }
		  }
		  if(PxLabel0==PxLabel2){
		    if((curr_line[j]!=0) && (prev_line[j-step]!=0) && (curr_line[j]!=prev_line[j-step]))
		      {
			if(curr_line[j]<prev_line[j-step]){
			  pair<int,int> p(curr_line[j],prev_line[j-step]);
			  EquivSet[p]=PxLabel0;
			}else{
			  pair<int,int> p(prev_line[j-step],curr_line[j]);
			  EquivSet[p]=PxLabel0;
			}
			//Equivalence E(curr_line[j],prev_line[j-1],PxLabel0);
			//EquivSet.insert(E);
		      }
		    else if(curr_line[j]==0 && prev_line[j-step]!=0){
		      curr_line[j]=prev_line[j-step];
		      mBlobs[PxLabel0][curr_line[j]]->merge(i,j);
		    }
		  }
		  if(PxLabel0==PxLabel3){
		    if((curr_line[j]!=0) && (prev_line[j]!=0) && (curr_line[j]!=prev_line[j]))
		      {
			if(curr_line[j]<prev_line[j]){
			  pair<int,int> p(curr_line[j],prev_line[j]);
			  EquivSet[p]=PxLabel0;
			}else{
			  pair<int,int> p(prev_line[j],curr_line[j]);
			  EquivSet[p]=PxLabel0;
			}
			//Equivalence E(curr_line[j],prev_line[j],PxLabel0);
			//EquivSet.insert(E);
		      }
		    else if(curr_line[j]==0 && prev_line[j]!=0){
		      curr_line[j]=prev_line[j];
		      mBlobs[PxLabel0][curr_line[j]]->merge(i,j); 
		    }
		  }
		  if(PxLabel0 != PxLabel1 && PxLabel0 != PxLabel2 && PxLabel0 != PxLabel3){
		    mBlobs[PxLabel0][nextBlobId] = new Blob(nextBlobId,i,j);
		    curr_line[j]=nextBlobId++;
		  }
		}
	    }
	  else //################################################################################ Generic Case
	  {
	    PxLabel0 = (char)mColorImage[mWidth*i + j];
	    if(ValidPixel(PxLabel0))
	      {
		PxLabel1=(char)mColorImage[mWidth*i + (j-step)];
		PxLabel2=(char)mColorImage[mWidth*(i-step) + (j-step)];
		PxLabel3=(char)mColorImage[mWidth*(i-step) + j];
		PxLabel4=(char)mColorImage[mWidth*(i-step) + (j+step)];
		if(PxLabel0==PxLabel1){
		  if((curr_line[j]!=0 && (curr_line[j-step]!=0) && (curr_line[j]!=curr_line[j-step]))) 
		    {
		      if(curr_line[j]<curr_line[j-step]){
			pair<int,int> p(curr_line[j],curr_line[j-step]);
			EquivSet[p]=PxLabel0;
		      }else{
			pair<int,int> p(curr_line[j-step],curr_line[j]);
			EquivSet[p]=PxLabel0;
		      }
		      //Equivalence E(curr_line[j],curr_line[j-1],PxLabel0);
		      //EquivSet.insert(E);
		    }
		  else if(curr_line[j]==0 && curr_line[j-step]!=0 ){
		    curr_line[j]=curr_line[j-step];
		    mBlobs[PxLabel0][curr_line[j]]->merge(i,j);
		  }
		}
		if(PxLabel0==PxLabel2){
		  if((curr_line[j]!=0) && (prev_line[j-step]!=0) && (curr_line[j]!=prev_line[j-step])) 
		    {
		      if(curr_line[j]<prev_line[j-step]){
			pair<int,int> p(curr_line[j],prev_line[j-step]);
			EquivSet[p]=PxLabel0;
		      }else{
			pair<int,int> p(prev_line[j-step],curr_line[j]);
			EquivSet[p]=PxLabel0;
		      }
		      //Equivalence E(curr_line[j],prev_line[j-1],PxLabel0);
		      //EquivSet.insert(E);
		    }
		  else if(curr_line[j]==0 && prev_line[j-step]!=0){
		    curr_line[j]=prev_line[j-step];
		    mBlobs[PxLabel0][curr_line[j]]->merge(i,j);
		  }
		}
		if(PxLabel0==PxLabel3){
		  if((curr_line[j]!=0) && (prev_line[j]!=0) && (curr_line[j]!=prev_line[j])) 
		    {
		      if(curr_line[j]<prev_line[j]){
			pair<int,int> p(curr_line[j],prev_line[j]);
			EquivSet[p]=PxLabel0;
		      }else{
			pair<int,int> p(prev_line[j],curr_line[j]);
			EquivSet[p]=PxLabel0;
		      }
		      //Equivalence E(curr_line[j],prev_line[j],PxLabel0);
		      //EquivSet.insert(E);
		    }
		  else if(curr_line[j]==0 && prev_line[j]!=0){
		    curr_line[j]=prev_line[j];
		    mBlobs[PxLabel0][curr_line[j]]->merge(i,j);
		  }
		}
		if(PxLabel0==PxLabel4){
		  if((curr_line[j]!=0) && (prev_line[j+1]!=0) && (curr_line[j]!=prev_line[j+1]))
		    {
		      if(curr_line[j]<prev_line[j+1]){
			pair<int,int> p(curr_line[j],prev_line[j+step]);
			EquivSet[p]=PxLabel0;
		      }else{
			pair<int,int> p(prev_line[j+step],curr_line[j]);
			EquivSet[p]=PxLabel0;
		      }
		      //Equivalence E(curr_line[j],prev_line[j+1],PxLabel0);
		      //EquivSet.insert(E);
		    }
		  else if(curr_line[j]==0 && prev_line[j+step]!=0){
		    curr_line[j]=prev_line[j+step];
		    mBlobs[PxLabel0][curr_line[j]]->merge(i,j);
		  }
		}
		if(PxLabel0!=PxLabel1 && PxLabel0!=PxLabel2 && PxLabel0!=PxLabel3 && PxLabel0!=PxLabel4){
		  mBlobs[PxLabel0][nextBlobId] = new Blob(nextBlobId,i,j);
		  curr_line[j]=nextBlobId++;
		}
	      }
	  }
	}
      prev_line.swap(curr_line);
      for(int k=0 ; k<mWidth ;k++){
	curr_line[k]=0;
      }
    }
  
  //#ifdef DEBUG
 // printf("\tTiming:: To do BlobGrowing \t= %d\n", getTime()-time);     
  //#endif 
  
  //#ifdef DEBUG
  /* time=getTime();*/
  //#endif
  NewEquiv.clear();
  NewEquiv.reserve(EquivSet.size());

  //  for(EquivItr=EquivSet.begin(); EquivItr!=EquivSet.end(); EquivItr++)
  //    NewEquiv.push_back(*EquivItr);

  for(EquivItr=EquivSet.begin(); EquivItr!=EquivSet.end(); EquivItr++)
    {    
      Equivalence E((EquivItr->first).first,(EquivItr->first).second,EquivItr->second);
      NewEquiv.push_back(E);
    }
  
  //#ifdef DEBUG
  //printf("\tTiming:: To create new equivalence \t= %d\n", getTime()-time);     
  //#endif 
  
  //Start new equivalence resolution
  //#ifdef DEBUG
  /* time = getTime(); */
  //#endif
  if(!NewEquiv.empty())
    {
      for(NewEquivItr1=NewEquiv.begin();NewEquivItr1!=NewEquiv.end();NewEquivItr1++)
	{
	  if(NewEquivItr1->GetFirst()!=NewEquivItr1->GetSecond())
	    {
	      mBlobs[NewEquivItr1->GetClass()][NewEquivItr1->GetFirst()]->merge(mBlobs[NewEquivItr1->GetClass()][NewEquivItr1->GetSecond()]);
	      for(NewEquivItr2=NewEquivItr1; NewEquivItr2!=NewEquiv.end(); NewEquivItr2++)
		{
		  if(NewEquivItr2!=NewEquivItr1)
		    {
		      if(NewEquivItr2->GetFirst() == NewEquivItr1->GetSecond())
			{
			  NewEquivItr2->SetFirst(NewEquivItr1->GetFirst());
			}
		      if(NewEquivItr2->GetSecond() == NewEquivItr1->GetSecond())
			{
			  NewEquivItr2->SetSecond(NewEquivItr2->GetFirst());
			  NewEquivItr2->SetFirst(NewEquivItr1->GetFirst());
			}
		    }
		}
	    }
	}
    }
  //#ifdef DEBUG
  //printf("\tTiming:: To Solve Equivalence \t= %d\n", getTime()-time);
  //#endif
}

/*
====================================================================================================================
====================================================================================================================
*/


int PixelMap::SaveBlobsToFile(char* filename,int filter)
{
  map < char, map< int,Blob* > >:: iterator BlobsItr1;
  map< int,Blob* > :: iterator BlobsItr2;
  
  int nchars=0;
  FILE * fd;
  fd = fopen(filename, "w");
  if (NULL == fd) return -1;
  
  for(BlobsItr1=mBlobs.begin();BlobsItr1!=mBlobs.end();BlobsItr1++)
    for(BlobsItr2 = BlobsItr1->second.begin();BlobsItr2!=BlobsItr1->second.end();BlobsItr2++)
      if(BlobsItr2->second->GetValid() && BlobsItr2->second->GetNumPix()>filter)
	{
	  nchars=fprintf(fd,"DEF_AREA %c %d %d %d %d %f %f %f %f %d\n",
			 BlobsItr1->first,
			 BlobsItr2->second->GetMinX(), 
			 BlobsItr2->second->GetMinY(),
			 BlobsItr2->second->GetMaxX(),
			 BlobsItr2->second->GetMaxY(),
			 BlobsItr2->second->GetMinRho(),
			 BlobsItr2->second->GetMaxRho(),
			 BlobsItr2->second->GetMinTheta(),
			 BlobsItr2->second->GetMaxTheta(),
			 BlobsItr2->second->GetNumPix());
	  if(nchars<0) return -1;
	}
  if(0!=fclose(fd)) 
    return -1;
  else
    return 0;
}


