 /* 
blob_detection.cpp

Author:  	Tim van Haren (timvanharen@live.com)
Date:		24 april, 2018
*/

#include "Arduino.h"
#include "Blob_detection.h"

Blobs::Blobs(){
}

void Blobs::make(int x, int y, int d){
	minx = x;
	maxx = x;
	miny = y;
	maxy = y;
	minDistance = d;
}

void Blobs::clearBlob(){
	minx = 0;
	maxx = 0;
	miny = 0;
	maxy = 0;
}

void Blobs::add(int x, int y){
	minx = min(minx, x);
    miny = min(miny, y);
    maxx = max(maxx, x);
    maxy = max(maxy, y);
}

int Blobs::isClose(int x, int y){

	// Find clamping point of x and y.
	int cx = max(min(maxx, x), minx);
	int cy = max(min(maxy, y), miny);
	
	// Find euclidian distance to nearest edge
	int distance = (x-cx)*(x-cx) + (y-cy)*(y-cy);
	
	// Check distance threshold
	if(distance <= minDistance * minDistance) 
		return TRUE;
	else 
		return FALSE;
}

int Blobs::blobSize() {
    return (maxx-minx) * (maxy-miny);
}

int Blobs::minX(){
	return minx;
}

int Blobs::maxX(){
	return maxx;
}

int Blobs::minY(){
	return miny;
}

int Blobs::maxY(){
	return maxy;
}

