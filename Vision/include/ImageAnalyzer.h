#ifndef IMAGEANALYZER_H_
#define IMAGEANALYZER_H_

#include "Vision/Results.h"

#include "ColorClassifier.h"
#include "ColorDataset.h"
#include "Blob.h"
#include "PixelMap.h"

#include "BlobBuffer.h"
#include "MovingAverageFilter.h"
#include "DistanceCalculator.h"

#define TOWER_CLASS 'R'
#define FACTORY_CLASS 'G'

#define MIN_BLOB_SIZE 200
#define MIN_BLOB_RATIO 0.17
#define MAX_BLOB_RATIO 0.63

class ImageAnalyzer
{
	private:
		KnnColorClassifier* cc;
		PixelMap* pm;
		ColorDataset* cd;
		BlobBuffer tower_buffer;
		BlobBuffer factory_buffer;
		MovingAverageFilter towerwidth_filter;
		MovingAverageFilter towerheight_filter;
		MovingAverageFilter factorywidth_filter;
		MovingAverageFilter factoryheight_filter;
		DistanceCalculator distanceCalculator;
	public:
		ImageAnalyzer(KnnColorClassifier* cc, PixelMap* pm, ColorDataset* cd);
		Vision::Results analyze(cv::Mat& img);
	private:
		void findObjects(cv::Mat& img);
		Vision::Results composeMessage();
		bool checkShape(int width, int height);
};

#endif /* IMAGEANALYZER_H_ */
