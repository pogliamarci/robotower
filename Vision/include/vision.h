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

#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"

#define MIN_BLOB_SIZE 100
#define BLOB_LIST_SIZE	15

using namespace cv;

// structs
struct blob_info
{
	int numPixel;
	char blobClass;
	Point a;
	Point b;
	Point center;
};

struct blobQueue
{
	struct blob_info list[BLOB_LIST_SIZE];
	int index;
};