 /* 
blob_detection.h header file 

Author:  	Tim van Haren (timvanharen@live.com)
Date:		24 april, 2018
*/

#ifndef Blob_detection_h
#define Blob_detection_h

#include "Arduino.h"

#define TRUE					1
#define FALSE					0


class Blobs
{
	public:
	Blobs();
	void make(int x, int y, int d);
	void clearBlob();
	void add(int x, int y);
	int isClose(int x, int y);
	int blobSize();
	int minX();
	int maxX();
	int minY();
	int maxY();

	private:
	int minDistance;
	int minx;
	int miny;
	int maxx;
	int maxy;
};

#endif Blob_detection_h