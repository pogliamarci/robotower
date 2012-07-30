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

#include <vector>
#include <iostream>
#include "BlobBuffer.h"

using namespace std;

BlobInfo* BlobBuffer::lastValidBlob()
{
	int current_index = index;
	BlobInfo* current_element;
	for (int i = 0; i < size; i++)
	{
		/* put current_index to the previous element */
		if (current_index == 0)
			current_index = size - 1;
		else
			current_index -= 1;
		/* returns the element if doesn't belong to the class 'U' */
		current_element = &(data->at(current_index));
		if (current_element->getClass() != 'U')
		{
			return current_element;
		}
	}
	return NULL; /* found no valid blob */
}

BlobBuffer::BlobBuffer(int buffer_size)
{
	index = 0;
	size = buffer_size;
	data = new vector<BlobInfo>(buffer_size);
}

void BlobBuffer::insert(BlobInfo element)
{
	data->at(index) = element;
	index += 1;
	if (index >= size) index = 0;
}

void BlobBuffer::addIfPresent(BlobInfo blob)
{
	if (blob.getNumPix() > 0)
	{
		this->insert(blob);
	}
	else
	{
		BlobInfo undefined_blob('U');
		this->insert(undefined_blob);
	}
}
