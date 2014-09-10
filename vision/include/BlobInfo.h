/*
 * BlobInfo.h
 *
 *  Created on: 30 Jul 2012
 *      Author: marcello
 */

#ifndef BLOBINFO_H_
#define BLOBINFO_H_

#include "opencv2/opencv.hpp"

class BlobInfo {
	private:
		int num_pixel;
		char blob_class;
		cv::Point center;
	public:
		cv::Point a;
		cv::Point b;
		BlobInfo(char new_class = 'U') : num_pixel(0), blob_class(new_class),
				center(-1,-1), a(-1,-1), b(-1,-1)
		{

		};
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
		inline void save(int num_pix, cv::Point point_1, cv::Point point_2)
		{
			this->num_pixel = num_pix;
			this->a = point_1;
			this->b = point_2;
			this->center.x = (point_2.x - point_1.x) / 2 + point_1.x;
			this->center.y = (point_2.y - point_1.y) / 2 + point_1.y;
		}

};

#endif /* BLOBINFO_H_ */
