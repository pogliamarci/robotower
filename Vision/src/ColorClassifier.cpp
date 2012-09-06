#include "ColorClassifier.h"
#include <stdlib.h>

bool operator==(const PointClass & first, const PointClass & second)
{
	return (first.getR() == second.getR() && first.getG() == second.getG()
			&& first.getB() == second.getB());
}

int operator<(const PointClass & first, const PointClass & second)
{
	if (first.getR() == second.getR())
		if (first.getG() == second.getG())
			return first.getB() < second.getB();
		else
			return first.getG() < second.getG();
	else
		return first.getR() < second.getR();
}

ColorClassifier::ColorClassifier()
{
	ColorCompressionFactor = COLOR_COMPRESSION_FACTOR;
	Create_Matrix(ColorCompressionFactor);
}

void ColorClassifier::Create_Matrix(ColorCompressionType CCF)
{
	ColorCompressionFactor = CCF;
	ColorMatrix = (unsigned char ***) malloc(
			(LEVELS_PER_CHANNEL >> ColorCompressionFactor)
					* sizeof(unsigned char **));
	ColorMatrixYUV = (unsigned char ***) malloc(
			(LEVELS_PER_CHANNEL >> ColorCompressionFactor)
					* sizeof(unsigned char **));
	for (unsigned int i = 0; i < (LEVELS_PER_CHANNEL >> ColorCompressionFactor);
			i++)
	{
		ColorMatrix[i] = (unsigned char **) malloc(
				(LEVELS_PER_CHANNEL >> ColorCompressionFactor)
						* sizeof(unsigned char *));
		ColorMatrixYUV[i] = (unsigned char **) malloc(
				(LEVELS_PER_CHANNEL >> ColorCompressionFactor)
						* sizeof(unsigned char *));
		for (unsigned int j = 0;
				j < (LEVELS_PER_CHANNEL >> ColorCompressionFactor); j++)
		{
			ColorMatrix[i][j] = (unsigned char *) malloc(
					(LEVELS_PER_CHANNEL >> ColorCompressionFactor)
							* sizeof(unsigned char));
			ColorMatrixYUV[i][j] = (unsigned char *) malloc(
					(LEVELS_PER_CHANNEL >> ColorCompressionFactor)
							* sizeof(unsigned char));
		}
	}

	for (unsigned int r = 0; r < (LEVELS_PER_CHANNEL >> ColorCompressionFactor);
			r++)
		for (unsigned int g = 0;
				g < (LEVELS_PER_CHANNEL >> ColorCompressionFactor); g++)
			for (unsigned int b = 0;
					b < (LEVELS_PER_CHANNEL >> ColorCompressionFactor); b++)
			{
				ColorMatrix[r >> ColorCompressionFactor][g
						>> ColorCompressionFactor][b >> ColorCompressionFactor] =
						'U';
				ColorMatrixYUV[r >> ColorCompressionFactor][g
						>> ColorCompressionFactor][b >> ColorCompressionFactor] =
						'U';
			}
}

void ColorClassifier::Delete_Matrix()
{
	for (unsigned int i = 0;
			i
					< ((LEVELS_PER_CHANNEL >> ColorCompressionFactor)
							>> ColorCompressionFactor); i++)
	{
		for (unsigned int j = 0;
				j
						< ((LEVELS_PER_CHANNEL >> ColorCompressionFactor)
								>> ColorCompressionFactor); j++)
		{
			free(ColorMatrix[i][j]);
			free(ColorMatrixYUV[i][j]);
		}
		free(ColorMatrix[i]);
		free(ColorMatrixYUV[i]);
	}
	free(ColorMatrix);
	free(ColorMatrixYUV);
}

ColorClassifier::ColorClassifier(ColorCompressionType CCF)
{
	this->Create_Matrix(CCF);
}

ColorClassifier::~ColorClassifier()
{
	this->Delete_Matrix();
}

void ColorClassifier::save_matrix(const char *filename)
{
	FILE *myfile = fopen(filename, "w");

	fprintf(myfile, "%d\n", (int) ColorCompressionFactor);

	for (unsigned int r = 0; r < (LEVELS_PER_CHANNEL >> ColorCompressionFactor);
			r++)
		for (unsigned int g = 0;
				g < (LEVELS_PER_CHANNEL >> ColorCompressionFactor); g++)
		{
			for (unsigned int b = 0;
					b < (LEVELS_PER_CHANNEL >> ColorCompressionFactor); b++)
				fputc(ColorMatrix[r][g][b], myfile);
			fprintf(myfile, "\n");
		}
	fclose(myfile);
}

void ColorClassifier::load_matrix(const char *filename)
{
	char trash[256];
	int ccf;
	FILE *myfile = fopen(filename, "r");

	fgets((char *) trash, 256, myfile);
	sscanf(trash, "%d", &ccf);
	this->Delete_Matrix();
	this->Create_Matrix((ColorCompressionType) ccf);

	for (unsigned int r = 0; r < (LEVELS_PER_CHANNEL >> ColorCompressionFactor);
			r++)
		for (unsigned int g = 0;
				g < (LEVELS_PER_CHANNEL >> ColorCompressionFactor); g++)
		{
			for (unsigned int b = 0;
					b < (LEVELS_PER_CHANNEL >> ColorCompressionFactor); b++)
				ColorMatrix[r][g][b] = fgetc(myfile);
			fgets((char *) trash, 256, myfile);
		}
	fclose(myfile);
}

void ColorClassifier::buildYUV()
{
	int pixel32;
	unsigned char *pixel = (unsigned char *) &pixel32;

	for (unsigned int y = 0; y < LEVELS_PER_CHANNEL; y++)
		for (unsigned int u = 0; u < LEVELS_PER_CHANNEL; u++)
			for (unsigned int v = 0; v < LEVELS_PER_CHANNEL; v++)
			{
				//	  printf("%d, %d, %d ",y,u,v);
				pixel32 = ColorSample::yuv2rgb((unsigned char) y,
						(unsigned char) u, (unsigned char) v);
				//	  printf(" %d ",pixel32);
				//	  printf("%d, %d, %d\n",pixel[0], pixel[1], pixel[2]);
				ColorMatrixYUV[y >> ColorCompressionFactor][u
						>> ColorCompressionFactor][v >> ColorCompressionFactor] =
						ColorMatrix[pixel[0] >> ColorCompressionFactor][pixel[1]
								>> ColorCompressionFactor][pixel[2]
								>> ColorCompressionFactor];
			}
}

void DbscanColorClassifier::build(ColorDataset * Dataset, int Eps,
		unsigned int NEps)
{
	int SampleDistance = 0, RDist = 0, GDist = 0, BDist = 0;
	bool Same = false;
	vector<unsigned char> NeighborClass;
	vector<unsigned char>::iterator ClassItr;

	map<unsigned char, unsigned int> Counters;
	map<unsigned char, unsigned int>::iterator CountItr;
	unsigned char MaxClass = 'U';
	unsigned int MaxCount = 0;

	set<ColorSample> CompressedColorSamples;
	set<ColorSample>::iterator SamplesIterator;
	set<ColorSample>::iterator ZeroDistIterator;

	set<ColorSample> NeighborHood; //TODO in a faster way!!!!!!!!
	set<ColorSample>::iterator NeiHoodIterator;

	ColorSample TmpColorSample;
	for (SamplesIterator = Dataset->begin(); SamplesIterator != Dataset->end();
			SamplesIterator++)
	{
		TmpColorSample = *SamplesIterator;
		TmpColorSample.setSamplingId(0);
		CompressedColorSamples.insert(TmpColorSample);
	}

	for (unsigned int r = 0; r < CELLS_PER_COLOR; r++)
	{
		for (unsigned int g = 0; g < CELLS_PER_COLOR; g++)
		{
			for (unsigned int b = 0; b < CELLS_PER_COLOR; b++)
			{

				NeighborClass.clear();

				// This might be a BUG!!!!
				//              ZeroDistIterator=NULL;
				Same = false;
				ColorMatrix[r][g][b] = 'U';

				Counters['R'] = 0;
				Counters['B'] = 0;
				Counters['Y'] = 0;
				Counters['G'] = 0;
				Counters['K'] = 0;
				Counters['W'] = 0;
				Counters['M'] = 0;
				Counters['C'] = 0;
				Counters['U'] = 0;
				MaxClass = 'U';
				MaxCount = 0;

				for (SamplesIterator = CompressedColorSamples.begin();
						SamplesIterator != CompressedColorSamples.end();
						SamplesIterator++)
				{

					SampleDistance = 0;

					RDist = abs(
							(int) (r
									- (unsigned int) SamplesIterator->getR()
											/ COLOR_COMPRESSION_FACTOR));
					GDist = abs(
							(int) (g
									- (unsigned int) SamplesIterator->getG()
											/ COLOR_COMPRESSION_FACTOR));
					BDist = abs(
							(int) (b
									- (unsigned int) SamplesIterator->getB()
											/ COLOR_COMPRESSION_FACTOR));

					SampleDistance = RDist + GDist + BDist;

					if (SampleDistance < Eps)
					{
						if (SampleDistance == 0)
						{
							ZeroDistIterator = SamplesIterator;
							Same = true;
						}

						NeighborClass.push_back(SamplesIterator->getClass());
						NeighborHood.insert(*SamplesIterator);
					}

				}

				if (NeighborClass.size() >= NEps)
				{
					for (ClassItr = NeighborClass.begin();
							ClassItr != NeighborClass.end(); ClassItr++)
						Counters[*ClassItr]++;

					for (CountItr = Counters.begin();
							CountItr != Counters.end(); CountItr++)
					{
						if (CountItr->second > MaxCount)
						{
							MaxCount = CountItr->second;
							MaxClass = CountItr->first;
						}
					}

					ColorMatrix[r][g][b] = MaxClass;
					//if(Same==true)
					//ZeroDistIterator->setClass( MaxClass );
				}

				//try to implement the concept of borders points
				else if (NeighborClass.size() > 0)
				{

					int count = 0;
					for (NeiHoodIterator = NeighborHood.begin();
							NeiHoodIterator != NeighborHood.end();
							NeiHoodIterator++)
					{
						count = 0;
						for (SamplesIterator = CompressedColorSamples.begin();
								SamplesIterator != CompressedColorSamples.end();
								SamplesIterator++)
						{
							SampleDistance = 0;

							RDist =
									abs(
											(int) (r
													- (unsigned int) SamplesIterator->getR()
															/ COLOR_COMPRESSION_FACTOR));
							GDist =
									abs(
											(int) (g
													- (unsigned int) SamplesIterator->getG()
															/ COLOR_COMPRESSION_FACTOR));
							BDist =
									abs(
											(int) (b
													- (unsigned int) SamplesIterator->getB()
															/ COLOR_COMPRESSION_FACTOR));

							SampleDistance = RDist + GDist + BDist;

							if (SampleDistance < Eps) count++;
						}

						if (count >= (int) NEps)
						{
							for (ClassItr = NeighborClass.begin();
									ClassItr != NeighborClass.end(); ClassItr++)
								Counters[*ClassItr]++;

							for (CountItr = Counters.begin();
									CountItr != Counters.end(); CountItr++)
							{
								if (CountItr->second > MaxCount)
								{
									MaxCount = CountItr->second;
									MaxClass = CountItr->first;
								}
							}

							ColorMatrix[r][g][b] = MaxClass;
							break;
						}
					}
					if ((Same == true) && (count <= (int) NEps))
						CompressedColorSamples.erase(ZeroDistIterator);
				}
				else if (Same == true)
					CompressedColorSamples.erase(ZeroDistIterator);

			} //end for b
		} //end for g
	} //end for r

	buildYUV();
	return;
}

void DbscanColorClassifier::fast_build(ColorDataset * Dataset, int Eps,
		unsigned int NEps, bool SEEDS)
{
	int SampleDistance = 0, RDist = 0, GDist = 0, BDist = 0;
	unsigned int count = 0;

	set<ColorSample> CompressedColorSamples;
	set<ColorSample>::iterator SamplesIterator;
	set<ColorSample>::iterator NearIterator;

	ColorSample TmpColorSample;
	for (SamplesIterator = Dataset->begin(); SamplesIterator != Dataset->end();
			SamplesIterator++)
	{
		TmpColorSample = *SamplesIterator;
		TmpColorSample.setSamplingId(0);
		CompressedColorSamples.insert(TmpColorSample);
	}

	for (unsigned int r = 0; r < (LEVELS_PER_CHANNEL >> ColorCompressionFactor);
			r++)
		for (unsigned int g = 0;
				g < (LEVELS_PER_CHANNEL >> ColorCompressionFactor); g++)
			for (unsigned int b = 0;
					b < (LEVELS_PER_CHANNEL >> ColorCompressionFactor); b++)
				ColorMatrix[r][g][b] = 'U';

	for (SamplesIterator = CompressedColorSamples.begin();
			SamplesIterator != CompressedColorSamples.end(); SamplesIterator++)
	{
		count = 0;

		for (NearIterator = CompressedColorSamples.begin();
				NearIterator != CompressedColorSamples.end(); NearIterator++)
		{
			SampleDistance = 0;

			RDist = abs(
					(int) ((unsigned int) NearIterator->getR()
							>> (ColorCompressionFactor
									- (unsigned int) SamplesIterator->getR())
							>> ColorCompressionFactor));
			GDist = abs(
					(int) ((unsigned int) NearIterator->getG()
							>> (ColorCompressionFactor
									- (unsigned int) SamplesIterator->getG())
							>> ColorCompressionFactor));
			BDist = abs(
					(int) ((unsigned int) NearIterator->getB()
							>> (ColorCompressionFactor
									- (unsigned int) SamplesIterator->getB())
							>> ColorCompressionFactor));

			SampleDistance = RDist + GDist + BDist;

			if ((SampleDistance < Eps) && (SampleDistance != 0)
					&& (SamplesIterator->getClass() == NearIterator->getClass()))
				count++;

		}

		if (count >= NEps)
		{

			int Rx = SamplesIterator->getR() >> ColorCompressionFactor;
			int Gx = SamplesIterator->getG() >> ColorCompressionFactor;
			int Bx = SamplesIterator->getB() >> ColorCompressionFactor;

			int startR = Rx - Eps + 1;
			int endR = Rx + Eps;

			int startG = Gx - Eps + 1;
			int endG = Gx + Eps;

			int startB = Bx - Eps + 1;
			int endB = Bx + Eps;

			if (startR < 0)
				startR = 0;
			else if (endR
					>= (int) (LEVELS_PER_CHANNEL >> ColorCompressionFactor))
				endR = (LEVELS_PER_CHANNEL >> ColorCompressionFactor) - 1;

			if (startG < 0)
				startG = 0;
			else if (endG
					>= (int) (LEVELS_PER_CHANNEL >> ColorCompressionFactor))
				endG = (LEVELS_PER_CHANNEL >> ColorCompressionFactor) - 1;

			if (startB < 0)
				startB = 0;
			else if (endB
					>= (int) (LEVELS_PER_CHANNEL >> ColorCompressionFactor))
				endB = (LEVELS_PER_CHANNEL >> ColorCompressionFactor) - 1;

			for (int r = startR; r <= endR; r++)
			{
				for (int g = startG; g <= endG; g++)
				{
					for (int b = startB; b <= endB; b++)
					{
						if ((abs(r - Rx) + abs(g - Gx) + abs(b - Bx)) <= Eps)
						{
							ColorMatrix[r][g][b] = SamplesIterator->getClass();
						}
					}
				}
			}

		}
		else if (count == 0) CompressedColorSamples.erase(SamplesIterator);

	} //fine tutto ciclo
	buildYUV();
	//fine metodo 
}
;

void KnnColorClassifier::build(ColorDataset * Dataset, int MaxDistance,
		unsigned int K)
{
	int SampleDistance = 0, RDist = 0, GDist = 0, BDist = 0;
	vector<int> NeighborDistance;
	vector<int>::iterator DistItr;
	vector<unsigned char> NeighborClass;
	vector<unsigned char>::iterator ClassItr;
	vector<int>::iterator FarDistItr;
	vector<unsigned char>::iterator FarClassItr;

	map<unsigned char, unsigned int> Counters;
	map<unsigned char, unsigned int>::iterator CountItr;
	unsigned char MaxClass = 'U';
	unsigned int MaxCount = 0;

	Counters['R'] = 0;
	Counters['B'] = 0;
	Counters['Y'] = 0;
	Counters['G'] = 0;
	Counters['K'] = 0;
	Counters['W'] = 0;
	Counters['M'] = 0;
	Counters['C'] = 0;
	Counters['U'] = 0;

	// Compress the dataset by removing SamplingId information
	set<ColorSample> CompressedColorSamples;
	set<ColorSample>::iterator SamplesIterator;

	ColorSample TmpColorSample;
	for (SamplesIterator = Dataset->begin(); SamplesIterator != Dataset->end();
			SamplesIterator++)
	{
		TmpColorSample = *SamplesIterator;
		TmpColorSample.setSamplingId(0);
		CompressedColorSamples.insert(TmpColorSample);
	}

	// If required add the seeds for the KCC Algorithm
	/* if(SEEDS)
	 * {
	 * CompressedColorSamples.insert(ColorSample(255,0,0,'R',0));
	 * CompressedColorSamples.insert(ColorSample(0,255,0,'G',0));
	 * CompressedColorSamples.insert(ColorSample(0,0,255,'B',0));
	 *
	 * CompressedColorSamples.insert(ColorSample(255,255,0,'Y',0));
	 * CompressedColorSamples.insert(ColorSample(255,0,255,'M',0));
	 * CompressedColorSamples.insert(ColorSample(0,255,255,'C',0));
	 *
	 * CompressedColorSamples.insert(ColorSample(0,0,0,'K',0));
	 * CompressedColorSamples.insert(ColorSample(255,255,255,'W',0));
	 * } */

	// Build up the Color Classification look-up table
	for (unsigned int r = 0; r < CELLS_PER_COLOR; r++)
	{
		for (unsigned int g = 0; g < CELLS_PER_COLOR; g++)
		{
			for (unsigned int b = 0; b < CELLS_PER_COLOR; b++)
			{
				// TODO:: Remove cell initialization
				ColorMatrix[r][g][b] = 'U';

				// Start the dummy KNN Algorithm with Max Distance
				if (!CompressedColorSamples.empty())
				{
					NeighborDistance.clear();
					NeighborClass.clear();

					//TODO:: write this in a nice way!
					for (SamplesIterator = CompressedColorSamples.begin();
							SamplesIterator != CompressedColorSamples.end();
							SamplesIterator++)
					{
						// Using Manhattan distance ...
						SampleDistance = 0;

						RDist = abs(
								(int) (r
										- (unsigned int) SamplesIterator->getR()
												/ COLOR_COMPRESSION_FACTOR));
						GDist = abs(
								(int) (g
										- (unsigned int) SamplesIterator->getG()
												/ COLOR_COMPRESSION_FACTOR));
						BDist = abs(
								(int) (b
										- (unsigned int) SamplesIterator->getB()
												/ COLOR_COMPRESSION_FACTOR));

						SampleDistance = RDist + GDist + BDist;

						// Neighbor computation if
						if (MaxDistance == 0 || SampleDistance < MaxDistance)
						{
							if (NeighborDistance.size() < K)
							{
								NeighborDistance.push_back(SampleDistance);
								NeighborClass.push_back(
										SamplesIterator->getClass());
							}
							else
							{
								FarDistItr = NeighborDistance.begin();
								FarClassItr = NeighborClass.begin();
								for (DistItr = NeighborDistance.begin();
										DistItr != NeighborDistance.end();
										DistItr++)
								{
									if (*FarDistItr < *DistItr)
									{
										FarDistItr = DistItr;
										FarClassItr = ClassItr;
									} // end if
									ClassItr++;
								} // end for
								if (SampleDistance < *FarDistItr)
								{
									*FarDistItr = SampleDistance;
									*FarClassItr = SamplesIterator->getClass();
								} // end if
							} // end if-else
						} // end if
					} // end for

					//Look for the most probable class ...
					Counters['R'] = 0;
					Counters['B'] = 0;
					Counters['Y'] = 0;
					Counters['G'] = 0;
					Counters['K'] = 0;
					Counters['W'] = 0;
					Counters['M'] = 0;
					Counters['C'] = 0;
					Counters['U'] = 0;
					MaxClass = 'U';
					MaxCount = 0;

					for (ClassItr = NeighborClass.begin();
							ClassItr != NeighborClass.end(); ClassItr++)
						Counters[*ClassItr]++;

					for (CountItr = Counters.begin();
							CountItr != Counters.end(); CountItr++)
					{
						if (CountItr->second > MaxCount)
						{
							MaxCount = CountItr->second;
							MaxClass = CountItr->first;
						}
					}

					ColorMatrix[r][g][b] = MaxClass;
				} //end if
			} // end for
		} // end for
		  // cout << r << " of " << CELLS_PER_COLOR << endl;
	} // end for

	buildYUV();
}

//KnnClassifier::KnnClassifier() : Classifer()

void KnnColorClassifier::fast_build(ColorDataset * Dataset, int MaxDistance,
		unsigned int K, bool SEEDS)
{
	unsigned int Index = 0;
	PointClass *pPointClass;
	map<PointClass, unsigned char> OutputClasses;
	map<PointClass, unsigned char>::iterator OutputClassesIterator;
	map<unsigned char, unsigned int> Counters;
	map<unsigned char, unsigned int>::iterator CountItr;
	unsigned char MaxClass = 'U';
	unsigned int MaxCount = 0;

	ColorSample TmpColorSample;
	set<ColorSample> CompressedColorSamples;
	set<ColorSample>::iterator SamplesIterator;

	// Compress the dataset by removing SamplingId information
	for (SamplesIterator = Dataset->begin(); SamplesIterator != Dataset->end();
			SamplesIterator++)
	{
		TmpColorSample = *SamplesIterator;
		TmpColorSample.setSamplingId(0);
		CompressedColorSamples.insert(TmpColorSample);
	}

	// If required add the seeds for the KCC Algorithm
	if (SEEDS)
	{
		CompressedColorSamples.insert(ColorSample(255, 0, 0, 'R', 0, RGB24));
		OutputClasses[PointClass(255 / COLOR_COMPRESSION_FACTOR, 0, 0)] = 'R';

		CompressedColorSamples.insert(ColorSample(0, 255, 0, 'G', 0, RGB24));
		OutputClasses[PointClass(0, 255 / COLOR_COMPRESSION_FACTOR, 0)] = 'G';

		CompressedColorSamples.insert(ColorSample(0, 0, 255, 'B', 0, RGB24));
		OutputClasses[PointClass(0, 0, 255 / COLOR_COMPRESSION_FACTOR)] = 'B';

		CompressedColorSamples.insert(ColorSample(255, 255, 0, 'Y', 0, RGB24));
		OutputClasses[PointClass(255 / COLOR_COMPRESSION_FACTOR,
				255 / COLOR_COMPRESSION_FACTOR, 0)] = 'Y';

		CompressedColorSamples.insert(ColorSample(255, 0, 255, 'M', 0, RGB24));
		OutputClasses[PointClass(255 / COLOR_COMPRESSION_FACTOR, 0,
				255 / COLOR_COMPRESSION_FACTOR)] = 'M';

		CompressedColorSamples.insert(ColorSample(0, 255, 255, 'C', 0, RGB24));
		OutputClasses[PointClass(0, 255 / COLOR_COMPRESSION_FACTOR,
				255 / COLOR_COMPRESSION_FACTOR)] = 'C';

		CompressedColorSamples.insert(ColorSample(0, 0, 0, 'K', 0, RGB24));
		OutputClasses[PointClass(0, 0, 0)] = 'K';

		CompressedColorSamples.insert(
				ColorSample(255, 255, 255, 'W', 0, RGB24));
		OutputClasses[PointClass(255 / COLOR_COMPRESSION_FACTOR,
				255 / COLOR_COMPRESSION_FACTOR, 255 / COLOR_COMPRESSION_FACTOR)] =
				'W';
	}

	// Initialize the ANN code. TODO: deallocate stuff!
	QueryPoint = annAllocPt(POINT_DIMENSIONS);
	DataPoints = annAllocPts(CompressedColorSamples.size(), POINT_DIMENSIONS);
	NnIdx = new ANNidx[K];
	Dists = new ANNdist[K];

	// Build up the Ann dataset and the corresponding output set
	for (SamplesIterator = CompressedColorSamples.begin();
			SamplesIterator != CompressedColorSamples.end(); SamplesIterator++)
	{
		DataPoints[Index][0] = ((unsigned short int) SamplesIterator->getR()
				>> ColorCompressionFactor);
		DataPoints[Index][1] = ((unsigned short int) SamplesIterator->getG()
				>> ColorCompressionFactor);
		DataPoints[Index][2] = ((unsigned short int) SamplesIterator->getB()
				>> ColorCompressionFactor);

		OutputClasses[PointClass((short int) DataPoints[Index][0],
				(short int) DataPoints[Index][1],
				(short int) DataPoints[Index][2])] =
				SamplesIterator->getClass();
		Index++;
	}

	// cout << OutputClasses.size() << "\t" << CompressedColorSamples.size() << endl;

	// Build up the Ann kd-tree
	KccTree = new ANNkd_tree(DataPoints, CompressedColorSamples.size(),
			POINT_DIMENSIONS);

	// Build up the ColorTable
	for (unsigned int r = 0; r < (LEVELS_PER_CHANNEL >> ColorCompressionFactor);
			r++)
	{
		for (unsigned int g = 0;
				g < (LEVELS_PER_CHANNEL >> ColorCompressionFactor); g++)
		{
			for (unsigned int b = 0;
					b < (LEVELS_PER_CHANNEL >> ColorCompressionFactor); b++)
			{
				// Retrieve the K nearest neihbors
				QueryPoint[0] = r;
				QueryPoint[1] = g;
				QueryPoint[2] = b;
				KccTree->annkSearch(QueryPoint, K, NnIdx, Dists, 0);

				//Look for the most probable class ...
				Counters['R'] = 0;
				Counters['B'] = 0;
				Counters['Y'] = 0;
				Counters['G'] = 0;
				Counters['K'] = 0;
				Counters['W'] = 0;
				Counters['M'] = 0;
				Counters['C'] = 0;
				Counters['U'] = 0;
				MaxClass = 'U';
				MaxCount = 0;

				for (Index = 0; Index < K; Index++)
				{
					if (Dists[Index] < MaxDistance || MaxDistance == 0)
					{
						pPointClass = new PointClass(
								(short int) DataPoints[NnIdx[Index]][0],
								(short int) DataPoints[NnIdx[Index]][1],
								(short int) DataPoints[NnIdx[Index]][2]);
						OutputClassesIterator = OutputClasses.find(
								*pPointClass);
						if (OutputClassesIterator != OutputClasses.end())
							Counters[OutputClassesIterator->second]++;
						delete (pPointClass);
					}
				}

				for (CountItr = Counters.begin(); CountItr != Counters.end();
						CountItr++)
				{
					if (CountItr->second > MaxCount)
					{
						MaxCount = CountItr->second;
						MaxClass = CountItr->first;
					}
				}

				ColorMatrix[r][g][b] = MaxClass;
			} // end b for
		} // end g for
	} // end r for

	// Clean up stuff
	annDeallocPt(QueryPoint);
	annDeallocPts(DataPoints);
	delete (NnIdx);
	delete (Dists);
	delete (KccTree);
	buildYUV();
}

