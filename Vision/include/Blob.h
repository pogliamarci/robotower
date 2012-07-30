/* Blob */

#ifndef BLOB_H
#define BLOB_H

#include <math.h>
#include "Receptor.h"

#define FULL_ANGLE 360
#define SMALL_ANGLE_THRESHOLD 30
#define POSITIVE_BIG_ANGLE_THRESHOLD 270
#define NEGATIVE_BIG_ANGLE_THRESHOLD -270

typedef enum
{
	NO_BLOB, PIXEL_BLOB, RECEPTOR_BLOB
} BlobType;

class Blob
{
	protected:
		int BlobID;
		int NumRec;
		int NumPix;

		BlobType type;

		int MinX;
		int MaxX;
		int MinY;
		int MaxY;
		float MinRho;
		float MaxRho;
		float MinTheta;
		float MaxTheta;
		bool Valid;
		bool FirstCrown;

	public:
		Blob(int BID) :
				NumRec(0), NumPix(0), type(NO_BLOB), MinX(-1), MaxX(-1), MinY(
						-1), MaxY(-1), MinRho(-1), MaxRho(-1), MinTheta(-1), MaxTheta(
						-1), Valid(true), FirstCrown(false)
		{
			BlobID = BID;
		}

		Blob(int BID, int X, int Y, float R, float T) :
				NumRec(1), NumPix(0), type(RECEPTOR_BLOB), Valid(true), FirstCrown(
						false)
		{
			BlobID = BID;
			MinX = MaxX = X;
			MinY = MaxY = Y;
			MinRho = MaxRho = R;
			MinTheta = MaxTheta = T;
		}

		Blob(int BID, int i, int j) :
				NumRec(0), NumPix(1), type(PIXEL_BLOB), Valid(true), FirstCrown(
						false)
		{
			BlobID = BID;
			MinX = MaxX = j;
			MinY = MaxY = i;
		}

		int GetMinX() const
		{
			return MinX;
		}
		int GetMinY() const
		{
			return MinY;
		}
		int GetMaxX() const
		{
			return MaxX;
		}
		int GetMaxY() const
		{
			return MaxY;
		}

		float GetMinRho() const
		{
			return MinRho;
		}
		void SetMinRho(float MR)
		{
			MinRho = MR;
		}

		float GetMinTheta() const
		{
			return MinTheta;
		}
		float GetMaxRho() const
		{
			return MaxRho;
		}
		float GetMaxTheta() const
		{
			return MaxTheta;
		}

		int GetBlobID() const
		{
			return BlobID;
		}
		int GetNumRec() const
		{
			return NumRec;
		}
		int GetNumPix() const
		{
			return NumPix;
		}

		bool GetValid() const
		{
			return Valid;
		}
		void SetValid(bool V)
		{
			Valid = V;
		}

		bool GetFirstCrown() const
		{
			return FirstCrown;
		}
		void SetFirstCrown(bool FC)
		{
			FirstCrown = FC;
		}

		void merge(int, int);
		void merge(Receptor*);
		void merge(Blob*);
};

class Equivalence
{
	public:
		Equivalence(int F, int S, char C) :
				mFirst(F), mSecond(S), mClass(C)
		{
		}
		;
		~Equivalence()
		{
		}
		;

		int GetFirst() const;
		void SetFirst(int F);
		int GetSecond() const;
		void SetSecond(int S);
		char GetClass() const;
		void SetClass(char C);

	protected:
		int mFirst;
		int mSecond;
		char mClass;
};

int operator<(const Equivalence&, const Equivalence&);
int operator<(const Blob&, const Blob&);

inline int Equivalence::GetFirst() const
{
	return mFirst;
}
;
inline void Equivalence::SetFirst(int F)
{
	mFirst = F;
}
;
inline int Equivalence::GetSecond() const
{
	return mSecond;
}
;
inline void Equivalence::SetSecond(int S)
{
	mSecond = S;
}
;
inline char Equivalence::GetClass() const
{
	return mClass;
}
;
inline void Equivalence::SetClass(char C)
{
	mClass = C;
}
;

#endif
