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

#define BLOB_LIST_SIZE	10

class BlobInfo {
	private:
		int num_pixel;
		char blob_class;
		cv::Point center;
	public:
		cv::Point a;
		cv::Point b;
		BlobInfo(char new_class = 'U');
		void save(int num_pix, cv::Point point_1, cv::Point point_2);
		inline char getClass() {
			return this->blob_class;
		}
		inline int getNumPix() {
			return this->num_pixel;
		}
		inline int getPosition() {
			return this->center.x;
		}
		inline int getWidth() {
			return this->b.x - this->a.x;
		}
		inline int getHeight() {
			return this->a.y - this->b.y;
		}
};

class BlobBuffer {
	private:
		std::vector<BlobInfo>* data;
		int size;
		int index;
	public:
		BlobBuffer(int buffer_size = BLOB_LIST_SIZE);
		void insert(BlobInfo element);
		BlobInfo* lastValidBlob();
		void addIfPresent(BlobInfo blob);
};

#endif
