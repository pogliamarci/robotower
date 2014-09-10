#ifndef CLASSIFIER_H
#define CLASSIFIER_H

//this class implements Color Classifiers
#include <vector>
#include <map>
#include <stdio.h>

#include "ANN/ANN.h"
#include "ColorDataset.h"

using namespace std;

typedef enum
{
	NO_COMPRESSION,
	COMPRESSION_TWO,
	COMPRESSION_FOUR,
	COMPRESSION_EIGHT,
	COMPRESSION_SIXTEEN
} ColorCompressionType;
const unsigned int LEVELS_PER_CHANNEL = 256;
const ColorCompressionType COLOR_COMPRESSION_FACTOR = COMPRESSION_TWO; // Use a power of 2 ; instead of moltiplication we use a shift
const unsigned int CELLS_PER_COLOR = LEVELS_PER_CHANNEL
		<< COLOR_COMPRESSION_FACTOR;
const unsigned int POINT_DIMENSIONS = 3; //new 

/** The Point Class is used to manage the class associated to the nearest neighbors. 
 */
class PointClass
{
		short int R;
		short int G;
		short int B;

	public:
		PointClass() :
				R(0), G(0), B(0)
		{
		}
		;
		PointClass(short int RR, short int GG, short int BB) :
				R(RR), G(GG), B(BB)
		{
		}
		;
		short int getR() const
		{
			return R;
		}
		short int getG() const
		{
			return G;
		}
		short int getB() const
		{
			return B;
		}
};

bool operator==(const PointClass&, const PointClass&);
int operator<(const PointClass&, const PointClass&);

/**This class finds and classifies the color clusters in the RGB Color Space.
 *Color Classifier is a general class that implements common features of two clustering algorithms. 
 *Two Algorithms are implemented : the classical Density Based Clusterig and the K Nearest Neighbors 
 */
class ColorClassifier
{
	protected:

		/** ColorMatrix implements the Color-LookUP-Table.
		 *This attribute is a dynamic 3D matrix, it rappresent the RGB color space, and maps the trasformation
		 *from RGB value to color label.
		 */
		unsigned char*** ColorMatrix;

		/** ColorMatrixYUV implements the YUV Color-LookUP-Table.
		 *This attribute is a dynamic 3D matrix, it rappresent the YUV color space, and maps the trasformation
		 *from YUB to RGB value and then to color label.
		 */
		unsigned char*** ColorMatrixYUV;

		/**ColorCompressionFactor stores the ColorMatrix compression.
		 * This attribute corresponds to the reduction of color depth used in the clustering algorithm.
		 * It is used by the class costructor to build the color matrix
		 */
		ColorCompressionType ColorCompressionFactor;

	public:

		/**Default Constructor with initialization. This Costructor inizializes color matrix with
		 * rank 64, ColorCompressionFactor equal to 2.
		 */
		ColorClassifier();

		/**Constructor with initialization and custom color compression factor.
		 *In this costructor the color compression factor is set to the input parameter
		 */
		ColorClassifier(ColorCompressionType);

		/**Distructor with color matrix deallocation.
		 */
		virtual ~ColorClassifier();

		void Create_Matrix(ColorCompressionType);
		void Delete_Matrix();
		ColorCompressionType Get_CCF()
		{
			return ColorCompressionFactor;
		}

		/**This function returns the label corresponding to RGB passed.
		 * The three int parameters identifie the index of the color matrix, the function
		 * returns the label stored in the matrix cell selected.
		 */
		unsigned char get_color(unsigned int R, unsigned int G, unsigned int B,
				ColorSpace CS = RGB24);

		/**This function save the color matrix in a txt file.
		 *this method  scans the matrix and, for each cell, stores the label in the file
		 *witch path is passed as the input parameter.
		 * @param filename is the name with the path of the file
		 */
		void save_matrix(const char* filename);

		/**This function load the color matrix from a txt file.
		 *This method inizializes the color matrix with the characters writter in file
		 *passed by the fuction.
		 * @param filename is the name with the path of the file
		 */
		void load_matrix(const char* filename);

		/**Build finds the clusters in RGB space setting labels in color matrix.
		 *This method is implemented in dother classes, ie DBSCAN and KNN. This
		 * function gets value from the class Dataset and using clustring algorithm
		 *"build" the Color LookUp Table. The parameters passed are respectively
		 * the search distance and the minimum number of neighbours
		 */
		virtual void build(ColorDataset*, int, unsigned int)
		{
		}
		;

		/** This is an optimezied version of method build
		 */
		virtual void fast_build(ColorDataset*, int, unsigned int, bool)
		{
		}
		;

		/** This is the build for the YUV classifier version
		 */
		void buildYUV();

};

/**This class implements the DBSCAN culstering algotithm.
 *DbscanClassifier inherits all attributes and methods from class Classifier, 
 *and reimplements the clustering methods, buil and fast_buil.
 */
class DbscanColorClassifier: public ColorClassifier
{
	public:

		/**This default costructor inherits from Classifier.
		 *See the Classifier costructor for more details
		 */
		DbscanColorClassifier() :
				ColorClassifier()
		{
		}

		/**This costructor inherits from Classifier with custom color compression factor.
		 *Setting the input parameter, the user can choose the color depth, ie the
		 *color matrix rank.
		 * @param CCF is the value of the color compression factor
		 */
		DbscanColorClassifier(ColorCompressionType CCF) :
				ColorClassifier(CCF)
		{
		}

		/**Build finds the clusters in RGB space setting labels in color matrix.
		 *This method implements the DBSCAN algorithm. This
		 * function gets value from the class Dataset and
		 *"build" the Color LookUp Table. The parameters passed are respectively
		 * the search distance and the minimum number of neighbours.
		 */
		void build(ColorDataset*, int, unsigned int);

		/**This is an optimized method of build.
		 */
		void fast_build(ColorDataset*, int, unsigned int, bool);

};

/**This class implements the KNN culstering algotithm.
 *KNNClassifier inherits all attributes and methods from class Classifier, 
 *and reimplements the clustering methods, build and fast_build.
 */
class KnnColorClassifier: public ColorClassifier
{
		/**This class stores all the informations used to build the KNNTree.
		 *Datapoints is a class of ANNpointArray type defined in the ANN library
		 *and contains objects of ANNpoint type.
		 */
		ANNpointArray DataPoints;
		/**This class is used by the KNNTree to store a point and find his
		 *nearest neighbors.
		 *QueryPoint is is a class of ANNpoint type defined in the ANN library
		 *and contains the RGB coordinates of the choosen point.
		 */
		ANNpoint QueryPoint;
		/**This class stores the indexes in the Datapoint class of the k nearest
		 *neighbors of the QueryPoint found with the function Search of the Class
		 *KNNTree.
		 *NnIdx is is a class of ANNidxArray type defined in the ANN library
		 *and contains the indexes of the k nearest neighbors of a choosen point.
		 */
		ANNidxArray NnIdx;
		/**This class stores the distances of the k nearest
		 *neighbors of the QueryPoint found with the function Search of the Class
		 *KNNTree.
		 *Dists is a class of ANNdistArray type defined in the ANN library
		 *and contains the distances of the k nearest neighbors of a choosen point
		 *and their RGB values could be found using the NnIdx class and the Datapoints
		 *class.
		 */
		ANNdistArray Dists;
		/**This class creates a tree with the Datapoints and implements a faster
		 *way to find the nearest neighbors.
		 *KccTree is a class of ANNkd_tree type defined in the ANN library and
		 *uses his function Search to find the nearest neighbors with a QueryPoint and
		 *k (number of nn) as input and returns the results of the computation in the
		 *in the NnIdx and Dists classes.
		 */
		ANNkd_tree *KccTree;

	public:

		/**This default costructor inherits from Classifier.
		 *See the Classifier costructor for more details.
		 */
		KnnColorClassifier() :
				ColorClassifier()
		{
		}

		/**This costructor inherits from Classifier with custom color compression factor.
		 *Setting the input parameter, the user can choose the color depth, ie the
		 *color matrix rank.
		 * @param CCF is the value of the color compression factor
		 */
		KnnColorClassifier(ColorCompressionType CCF) :
				ColorClassifier(CCF)
		{
		}

		/**Build finds the clusters in RGB space setting labels in color matrix.
		 *This method implements the KNN algorithm. This
		 * function gets value from the class Dataset and
		 *"build" the Color LookUp Table. The parameters passed are respectively
		 * the maximum distance and the number of neighbours.
		 */
		void build(ColorDataset*, int, unsigned int);

		/**fast_build finds the cluster in RGB space in a faster way using the ANN
		 *library.
		 */
		void fast_build(ColorDataset*, int, unsigned int, bool);

};

inline unsigned char ColorClassifier::get_color(unsigned int R, unsigned int G,
		unsigned int B, ColorSpace CS)
{
	if (CS == RGB24)
		return ColorMatrix[R >> ColorCompressionFactor][G
				>> ColorCompressionFactor][B >> ColorCompressionFactor];
	else if (CS == YUV411)
		return ColorMatrixYUV[R >> ColorCompressionFactor][G
				>> ColorCompressionFactor][B >> ColorCompressionFactor];
	else
		return 'U';
}
;

#endif
