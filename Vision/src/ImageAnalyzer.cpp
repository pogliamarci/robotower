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

#include "ImageAnalyzer.h"

Vision::Results ImageAnalyzer::analyze(cv::Mat& img)
{
	findObjects(img);

	/* draw result on image (debug) */
	BlobInfo* tower_result = tower_buffer.lastValidBlob();
	if (tower_result != NULL && tower_result->getClass() == TOWER_CLASS)
	{
		cv::rectangle(img, tower_result->a, tower_result->b, CV_RGB(254,254,0), 2, 8, 0);
	}
	BlobInfo* factory_result = factory_buffer.lastValidBlob();
	if (factory_result != NULL && factory_result->getClass() == FACTORY_CLASS)
	{
		cv::rectangle(img, factory_result->a, factory_result->b, CV_RGB(254,0,0), 2, 8, 0);
	}

	towerwidth_filter.update(tower_result != NULL ? tower_result->getWidth() : 0);
	towerheight_filter.update(tower_result != NULL ? tower_result->getHeight() : 0);
	factorywidth_filter.update(factory_result != NULL ? factory_result->getWidth() : 0);
	factoryheight_filter.update(factory_result != NULL ? factory_result->getHeight() : 0);
	distanceCalculator.insertDatiFabbrica((int)factoryheight_filter.curValue(), (int) factorywidth_filter.curValue());
	distanceCalculator.insertDatiTorre((int)towerheight_filter.curValue(), (int) towerwidth_filter.curValue());


	return composeMessage();
}

Vision::Results ImageAnalyzer::composeMessage()
{
	Vision::Results msg;

	BlobInfo* tower = tower_buffer.lastValidBlob();
	BlobInfo* factory = factory_buffer.lastValidBlob();

	msg.towerFound = tower != NULL;
	msg.towerPos = tower != NULL ? tower->getPosition() : 0;

	msg.towerBlobHeight = (int) towerheight_filter.curValue();
	msg.towerBlobWidth = (int) towerwidth_filter.curValue();
	msg.factoryBlobHeight = (int) factorywidth_filter.curValue();
	msg.factoryBlobWidth = (int) factoryheight_filter.curValue();

	msg.towerDistance = distanceCalculator.getDistanzaTorre();
	msg.factoryDistance = distanceCalculator.getDistanzaFabbrica();

	msg.factoryFound = factory != NULL;
	msg.factoryPos = factory != NULL ? factory->getPosition() : 0;

	return msg;
}

void ImageAnalyzer::findObjects(cv::Mat& img)
{
	/* Point 2D */
	cv::Point pt1, pt2;

	/* Maps needed for BlobAnalysis */
	map < char, map< int,Blob* > >:: iterator BlobsItr1;
	map< int,Blob* > :: iterator BlobsItr2;

	/* initialise classes to store data regarding the found blobs */
	BlobInfo tower_blob(TOWER_CLASS);
	BlobInfo factory_blob(FACTORY_CLASS);

	/* Blob parameters, PixMap object initialisation */
	pm->SetImage((unsigned char*)img.data, img.cols, img.rows, cc, 3);

	/* Start blob growing algorithm: parameters from the 2nd to the 5th
	 * identify the region of the image in which blobs are to be searched for
	 * (2nd to 5th params are set to -1 ==> search blobs over the whole image)
	 */
	pm->BlobGrowing(1,-1,-1,-1,-1);
	for(BlobsItr1=pm->GetBlobs()->begin();BlobsItr1!=pm->GetBlobs()->end();BlobsItr1++)
	{
   		for(BlobsItr2 = BlobsItr1->second.begin();BlobsItr2!=BlobsItr1->second.end();BlobsItr2++)
   		{
   			if(BlobsItr2->second->GetValid() && BlobsItr2->second->GetNumPix()>MIN_BLOB_SIZE)
			{
				/* load blob coordinates */
   				/* top left point */
				pt1.x = BlobsItr2->second->GetMinX();
				pt1.y = BlobsItr2->second->GetMaxY();
				/* bottom right point */
				pt2.x = BlobsItr2->second->GetMaxX();
				pt2.y = BlobsItr2->second->GetMinY();
				cout << "Blob found: " << BlobsItr1->first << endl;

				/* compute width and height to perform some check... */
				int blob_width = pt2.x - pt1.x;
				int blob_heigth = pt1.y - pt2.y;

				/* is the blob shape similar to the expected one? */
				/* save the biggest found blob for each colour class */
				if(BlobsItr1->first == TOWER_CLASS &&
						BlobsItr2->second->GetNumPix() > tower_blob.getNumPix() &&
						checkShape(blob_width, blob_heigth) )
				{
						tower_blob.save(BlobsItr2->second->GetNumPix(), pt1, pt2);
				}
				if(BlobsItr1->first == FACTORY_CLASS &&
						BlobsItr2->second->GetNumPix() > factory_blob.getNumPix() &&
						checkShape(blob_width, blob_heigth) )
				{
						factory_blob.save(BlobsItr2->second->GetNumPix(), pt1, pt2);
				}
			}
   		}
	}

	/* add results to buffer */
	tower_buffer.addIfPresent(tower_blob);
	factory_buffer.addIfPresent(factory_blob);
}

bool ImageAnalyzer::checkShape(int width, int heigth) {
	// Shape control -> must be rectangular with height > width
	float ratio = ((float) width) / ((float) heigth);
	cout << "Factory ratio" << ratio << endl;
	if (ratio > MIN_BLOB_RATIO && ratio < MAX_BLOB_RATIO)
	{
		return true;
	}
	return false;
}


ImageAnalyzer::ImageAnalyzer(KnnColorClassifier* cc, PixelMap* pm, ColorDataset* cd)
{
	this->cc = cc;
	this->pm = pm;
	this->cd = cd;
}
