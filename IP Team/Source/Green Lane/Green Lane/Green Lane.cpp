#include "stdafx.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <cv.h>
#include <highgui.h>
#include <math.h>
#include "tserial.h"

#define TEST 0
#define FORWARD 1

IplImage* DrawHistogram(int *bins, float scaleX=1, float scaleY=1)
{
	float histMax = 0;
    for(int i=0; i<256; i++)
	{
		if(bins[i] >= histMax)
		{
			histMax = (float)bins[i];
		}
	}

	IplImage* imgHist = cvCreateImage(cvSize(256*scaleX, 64*scaleY), 8 ,1);
    cvZero(imgHist);

	for(int i=0;i<255;i++)
    {
        float histValue = bins[i];
        float nextValue = bins[i+1];
 
		if((histValue!=0.0)&&(nextValue!=0.0))
		{
			CvPoint pt1 = cvPoint(i*scaleX, 64*scaleY);
			CvPoint pt2 = cvPoint(i*scaleX+scaleX, 64*scaleY);
			CvPoint pt3 = cvPoint(i*scaleX+scaleX, (64-nextValue*64/histMax)*scaleY);
			CvPoint pt4 = cvPoint(i*scaleX, (64-histValue*64/histMax)*scaleY);
 
			int numPts = 5;
			CvPoint pts[] = {pt1, pt2, pt3, pt4, pt1};
 
			cvFillConvexPoly(imgHist, pts, numPts, cvScalar(255));
		}
    }
	return imgHist;
}

int OtsuThreshold(int *bins, int nbins){
	int total = 256;
	float sum = 0;
	int t;
	for (t = 0; t < nbins; t++){
		sum += t * bins[t];
	}

	float sumB = 0;
	int wB = 0;
	int wF = 0;
	float varMax = 0;
	int threshold = 0;
	for (t = 0; t < nbins; t++){
		wB += bins[t];               // Weight Background
		if (wB == 0)	continue;
		wF = total - wB;                 // Weight Foreground
		if (wF == 0)	break;
		sumB += (float) (t * bins[t]);
		float mB = sumB / wB;            // Mean Background
		float mF = (sum - sumB) / wF;    // Mean Foreground
		float varBetween = (float)wB * (float)wF * (mB - mF) * (mB - mF);	// Calculate Between Class Variance
		// Check if new maximum found
		if (varBetween > varMax) {
			varMax = varBetween;
			threshold = t;
		}
	}
	fprintf(stdout, "otsu threshold=%d\n", threshold);
	
	return threshold;
}

IplImage* Laplacian(IplImage* in)
{
	int **A, **B;
	A = (int **) calloc(in->height, sizeof(int));
	for(int i=0; i<in->height; i++)
		A[i] = (int*) calloc(in->width, sizeof(int));
	B = (int **) calloc(in->height, sizeof(int));
	for(int i=0; i<in->height; i++)
		B[i] = (int*) calloc(in->width, sizeof(int));
	
	for(int i=0; i<in->height; i++)
	{
		uchar* ptr = (uchar*) (in->imageData + i*in->widthStep);
		for(int j=0; j<in->width; j++)
			A[i][j] = ptr[j];
	}
	float min=255, max=0;
	for(int i=1; i<in->height-1; i++)
		for(int j=1; j<in->width-1; j++)
		{
			B[i][j] = A[i-1][j] + A[i][j-1] + (-4)*A[i][j] + A[i][j+1] + A[i+1][j];
			if(B[i][j] <= min)
				min = (float)B[i][j];
			if(B[i][j] >= max)
				max = (float)B[i][j];
		}
	printf("max:%f min:%f\n", max, min);
	for(int i=1; i<in->height; i++)
		for(int j=1; j<in->width; j++)
			B[i][j] = (int)(B[i][j]-min)/(max-min)*255;
	IplImage *out = cvCreateImage(cvGetSize(in), 8, 1);
	for(int i=0; i<out->height; i++)
	{
		uchar* ptr = (uchar*) (out->imageData + i*out->widthStep);
		for(int j=0; j<out->width; j++)
			ptr[j] = B[i][j];
	}
	free(A);
	free(B);
	return out;
}

int PercentileThreshold(IplImage* in, float percentile)
{
	int bins[256];
	for(int i=0; i<256; i++)
		bins[i] = 0;
	
	for(int i=0; i<in->height; i++){	// Histogram construction
		uchar* ptr = (uchar*) (in->imageData + i*in->widthStep);
		for(int j=0; j<in->width; j++)
			bins[ptr[j]]++;							
	}
	int sum=0, i=255;
	while((i >= 0)&&(sum < in->imageSize*percentile/100))
		sum += bins[i--];
	printf("Percentile Threshold:%d\n", i);
	
	return i;
}

int send(int mode)
{
	Tserial* p = new Tserial();
    p->connect("COM2",9600,spNONE);
    switch(mode)
	{
	case TEST:
		p->sendChar('w');
		p->sendChar('5');
		p->sendChar('5');
		break;
	default:
		break;
	}
    p->disconnect();
	return 0;
}

int _tmain(int argc, _TCHAR* argv[])
{
	//CvCapture *capture = cvCaptureFromCAM(0);
	//IplImage *img = cvQueryFrame(capture);
	IplImage *img = cvLoadImage("../../Data/13 Oct/img4.jpg");
	cvNamedWindow("Original", 0);
	cvShowImage("Original", img);

	int t = 10;
	while(1)
	{
		//------------------------ Finding Gray Scale Image -------------------------------------//
		IplImage *gray = cvCreateImage(cvGetSize(img), 8, 1);
		cvCvtColor(img, gray, CV_BGR2GRAY);
		cvNamedWindow("Gray", 0);
		cvShowImage("Gray", gray);
		
		//------------------------ Removing Salt and Pepper Noise -------------------------------//
		//cvSmooth(gray, gray, CV_MEDIAN, 3, 3, 0.0, 0.0);		// For really poor images
		cvMorphologyEx(gray, gray, NULL, NULL, CV_MOP_OPEN, 1);	// Removes Salt Noise
		for(int y = 0; y < gray->height; y++){
			uchar* ptr = (uchar*) (gray->imageData + y * gray->widthStep);								//implement inversion and pepper removal in single pass
			for(int x = 0; x < gray->width; x++)
				ptr[x] = 255-ptr[x];							// Pepper becomes Salt
		}
		cvMorphologyEx(gray, gray, NULL, NULL, CV_MOP_OPEN, 1);	// Removes Pepper Noise
		cvShowImage("Gray", gray);
		//------------------------ Finding Gradient Image ---------------------------------------//
		IplImage *gradient = Laplacian(gray);
		cvNamedWindow("Gradient", 0);
		cvShowImage("Gradient", gradient);

		//------------ Applying 90 Percentile Threshold to get a Laplacian Edge Mask ------------//
		cvThreshold(gradient, gradient, (double)PercentileThreshold(gradient, (float)t), 255, CV_THRESH_BINARY);	// Percentile from top
		cvNamedWindow("BEImage", 0);
		cvShowImage("BEImage", gradient);
		
		//--------------- Applying the Laplacian Edge Mask to Original Gray Image ---------------//
		for(int y = 0; y < gray->height; y++){
			uchar* ptr = (uchar*) (gray->imageData + y * gray->widthStep);
			uchar* ptr1 = (uchar*) (gradient->imageData + y * gradient->widthStep);
			for(int x = 0; x < gray->width; x++){
				if(ptr1[x] == 0){
					ptr[x] = 0;
				}
			}
		}
		cvNamedWindow("Final", 0);
		cvShowImage("Final", gray);

		//---------------------- Making a Histogram of the Masked Gray Image --------------------//
		int bins[256];
		for(int i=0; i<256; i++)
		{
			bins[i] = 0;
		}
		for(int y = 0; y < gray->height; y++){
			uchar* ptr = (uchar*) (gray->imageData + y * gray->widthStep);
			for(int x = 0; x < gray->width; x++){
				if(ptr[x] != 0)
				{
					bins[ptr[x]]++;
				}
			}
		}
		IplImage* histimg = DrawHistogram(bins);
		cvNamedWindow("Histogram", 0);
		cvShowImage("Histogram", histimg);
		
		//------------- Thresholding the Original Gray Image using The Otsu Threshold -----------//
		IplImage *otsu = cvCreateImage(cvGetSize(img), 8, 1);
		cvCvtColor(img, otsu, CV_BGR2GRAY);
		cvThreshold(otsu, otsu, 255-OtsuThreshold(bins, 256), 255, CV_THRESH_BINARY);
		cvNamedWindow("Otsu Threshold", 0);
		cvShowImage("Otsu Threshold", otsu);
		
		for(int y = 0; y < otsu->height; y++){
			uchar* ptr = (uchar*) (otsu->imageData + y * otsu->widthStep);
			for(int x = 0; x < otsu->width; x++)
				if(ptr[x]==255)
				{
					int sum = 0, d = 10;
					for(int i=0; i<d; i++)
						ptr[x+i] = 0;
					ptr[x+d/2] = 255;
					break;
				}
		}
		cvShowImage("Otsu Threshold", otsu);

		for(int y = 0; y < otsu->height-3; y+=3)
		{
			uchar* ptr = (uchar*) (otsu->imageData + y * otsu->widthStep);
			uchar* ptr1 = (uchar*) (otsu->imageData + (y+1) * otsu->widthStep);
			uchar* ptr2 = (uchar*) (otsu->imageData + (y+2) * otsu->widthStep);
			for(int x = 0; x < otsu->width-3; x+=3)
			{
				if((ptr[x] + ptr[x+1] + ptr[x+2] + ptr1[x] + ptr1[x+1] + ptr1[x+2] + ptr2[x] + ptr2[x+1] + ptr2[x+2])/255 > 1)
				{
					ptr[x] = 0;		ptr[x+1] = 0;		ptr[x+2] = 0;
					ptr1[x] = 0;	ptr1[x+1] = 255;	ptr1[x+2] = 0;
					ptr2[x] = 0;	ptr2[x+1] = 0;		ptr2[x+2] = 0;
				}
				else
				{
					ptr[x] = 0;		ptr[x+1] = 0;		ptr[x+2] = 0;
					ptr1[x] = 0;	ptr1[x+1] = 0;		ptr1[x+2] = 0;
					ptr2[x] = 0;	ptr2[x+1] = 0;		ptr2[x+2] = 0;
				}
			}
		}
		cvShowImage("Otsu Threshold", otsu);
		//---------------------------------- Hough Lines ----------------------------------------//
		CvMemStorage* line_storage = cvCreateMemStorage(0);
		CvSeq* results =  cvHoughLines2(otsu, line_storage, CV_HOUGH_PROBABILISTIC, 5, CV_PI/180, 100, 100, 100);
		float angle = 0.0, temp;
		printf("lines:%d\n", results->total);
		for(int i = 0; i < results->total; i++)
		{
			CvPoint* line = (CvPoint*)cvGetSeqElem(results, i);
			cvLine(img, line[0], line[1], CV_RGB(255,0,0), 2, CV_AA, 0);
			if(line[0].y > line[1].y)
				temp = atan((line[0].y - line[1].y + 0.0) / (line[0].x - line[1].x));
			else
				temp = atan((line[1].y - line[0].y + 0.0) / (line[1].x - line[0].x));
			if(temp < 0)
				angle += 90 + 180/3.14*temp;
			else
				angle += 180/3.14*temp - 90;
		}
		printf("angle: %f\n", angle/results->total);
		cvNamedWindow("Hough Lines", 0);
		cvShowImage("Hough Lines", img);

		int c = cvWaitKey(0);
		if(c=='a')
			t++;
		else if(c == 'z')
			t--;
		else
		{
			
			break;
		}
	}
	
	return 0;
}

