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

	/* compute distance to the tower */
	towersize_filter.update(tower_result->getWidth());

	return composeMessage();
}

float ImageAnalyzer::getTowerDistance(float size) {
	const float real_twrsize = 1.0;
	const float focal_length = 1.0;
	return size * real_twrsize / focal_length;
}

Vision::Results ImageAnalyzer::composeMessage()
{
	Vision::Results msg;

	BlobInfo* tower = tower_buffer.lastValidBlob();
	BlobInfo* factory = factory_buffer.lastValidBlob();

	msg.towerFound = tower != NULL;
	msg.towerPos = tower != NULL ? tower->getPosition() : 0;
	/*
	 * msg.towerSize = tower != NULL : getTowerDistance(towersize_filter.curValue()) : 0;
	 * (meglio sarebbe qualcosa del tipo msg.towerSize = tower != NULL : tower->getSize : 0;)
	 */
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
						tower_blob.getNumPix() <  BlobsItr2->second->GetNumPix())
				{
					tower_blob.save(BlobsItr2->second->GetNumPix(), pt1, pt2);
				}
				if(BlobsItr1->first == FACTORY_CLASS &&
						factory_blob.getNumPix() <  BlobsItr2->second->GetNumPix())
				{
					if (checkShape(blob_width, blob_heigth))
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
	/* Shape control -> must be rectangular with height > width */
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
