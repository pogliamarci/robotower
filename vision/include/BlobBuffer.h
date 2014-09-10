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

#ifndef BLOBBUFFER_H
#define BLOBBUFFER_H

#include "BlobInfo.h"

#define DEFAULT_BLOB_LIST_SIZE	5

class BlobBuffer
{
	private:
		std::vector<BlobInfo>* data;
		int size;
		int index;
	public:
		BlobBuffer(int buffer_size = DEFAULT_BLOB_LIST_SIZE);
		void insert(BlobInfo element);
		BlobInfo* lastValidBlob();
		void addIfPresent(BlobInfo blob);
};

#endif
