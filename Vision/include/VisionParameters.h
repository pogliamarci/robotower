#ifndef VISIONPARAMETERS_H_
#define VISIONPARAMETERS_H_

#include "ImageAnalyzer.h"

class VisionParameters {
	private:
		KnnColorClassifier cc;
		/* PixelMap pm; */
		ColorDataset cd;
		ImageAnalyzer analyzer;
		char* dataset;
		char* classifier;
		static char defaultDataset[];
		static char defaultClassifier[];
	public:
		VisionParameters() : analyzer(&cc, &cd),
					dataset(defaultDataset), classifier(defaultClassifier) {}
		void buildClassifier();
		void loadClassifier();
		inline Vision::Results startAnalysis(cv::Mat& frame) {
			return analyzer.analyze(frame);
		}
		inline void setDataset(char* dataset) {
			this->dataset = dataset;
		}
		inline void setClassifier(char* classifier) {
			this->classifier = classifier;
		}
};

#endif /* VISIONPARAMETERS_H_ */
