#ifndef IMAGEANALYZER_H_
#define IMAGEANALYZER_H_

#include "SupportDataStructures.h"

#include "Vision/Results.h"

#include "ColorClassifier.h"
#include "ColorDataset.h"
#include "Blob.h"
#include "PixelMap.h"

#define TOWER_CLASS 'R'
#define FACTORY_CLASS 'G'

#define MIN_BLOB_SIZE 200
#define MIN_BLOB_RATIO 0.17
#define MAX_BLOB_RATIO 0.63

class MovingAverageFilter
{
	private:
		float alpha;
		float x_k;
	public:
		MovingAverageFilter()
		{
			alpha = 0.125;
		}
		inline float update(float x_knew)
		{
			x_k = alpha * x_knew + (1 - alpha) * x_k;
			return x_k;
		}
		inline float curValue()
		{
			return x_k;
		}
};

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
		float getTowerDistance(float size);
		void findObjects(cv::Mat& img);
		Vision::Results composeMessage();
		bool checkShape(int width, int height);
	public:
		Vision::Results analyze(cv::Mat& img);
		ImageAnalyzer(KnnColorClassifier* cc, PixelMap* pm, ColorDataset* cd);
};

#endif /* IMAGEANALYZER_H_ */
