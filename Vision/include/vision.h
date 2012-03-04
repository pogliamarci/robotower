/*
 * RoboTower, Hi-CoRG based on ROS
 *
 * Copyright (C) 2012 Politecnico di Milano
 * Copyright (C) 2012 Marcello Pogliani, Davide Tateo
 * Versione 1.0
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation, version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef VISION_H
#define VISION_H

#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "ColorClassifier.h"
#include "ColorDataset.h"
#include "Blob.h"
#include "PixelMap.h"

#define MIN_BLOB_SIZE 100
#define BLOB_LIST_SIZE	15
#define NUM_FRAME_TO_SEARCH 5
#define MIN_BLOB_RATIO 0.2
#define MAX_BLOB_RATIO 0.8

class BlobInfo 
{
	private:
		int num_pixel;
		char blob_class;
		cv::Point center;
	public:
		cv::Point a;
		cv::Point b;
		BlobInfo(char new_class = 'U');
		void save(int num_pix, cv::Point point_1, cv::Point point_2);
		inline char getClass() 
		{
			return this->blob_class;
		}
		inline int getNumPix() 
		{
			return this->num_pixel;
		}
};

class BlobBuffer 
{
	private:
		std::vector<BlobInfo>* data;
		int size;
		int index;
	public:
		BlobBuffer(int buffer_size = BLOB_LIST_SIZE);
		void insert(BlobInfo element);
		BlobInfo* lastValidBlob();
};

#endif
