#include "stdafx.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <cv.h>
#include <highgui.h>
#include <math.h>
#include "tserial.h"
#include <time.h>

#define TEST 0
#define FORWARD 1
#define BRAKE 2

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
	//fprintf(stdout, "otsu threshold=%d\n", threshold);
	
	return threshold;
}

int send(int mode, float angle)
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
		case FORWARD:
			p->sendChar('w');
			if(angle >= 10)
			{
				p->sendChar('3');
				p->sendChar('7');
			}
			else if(angle <= -10)
			{
				p->sendChar('7');
				p->sendChar('3');
			}
			else
			{
				p->sendChar('5');
				p->sendChar('5');
			}
			break;
		case BRAKE:
			p->sendChar('s');
			break;
		default:
			break;
	}
    p->disconnect();
	return 0;
}

double diffclock(clock_t clock1,clock_t clock2)
{
	double diffticks=clock1-clock2;
	double diffms=(diffticks*1000)/CLOCKS_PER_SEC;
	return diffms;
} 

int _tmain(int argc, _TCHAR* argv[])
{
	//CvCapture *capture = cvCaptureFromCAM(0);
	
	int t = 10, n=0;
	double time = 0;
	while(1)
	{
		clock_t begin=clock();
		//------------------------ Original Camera Feed -----------------------------------------//
		//IplImage *img = cvQueryFrame(capture);
		IplImage *img = cvLoadImage("../../Data/13 Oct/img13.jpg");
		//cvNamedWindow("Camera Feed", 0);
		//cvShowImage("Camera Feed", img);
				
		//------------------------ Finding Gray Scale Image -------------------------------------//
		IplImage *gray = cvCreateImage(cvGetSize(img), 8, 1);
		cvCvtColor(img, gray, CV_BGR2GRAY);
		//cvReleaseImage(&img);

		//------------------------ Removing Salt and Pepper Noise -------------------------------//
		//cvSmooth(gray, gray, CV_MEDIAN, 3, 3, 0.0, 0.0);		// For really poor images
		cvMorphologyEx(gray, gray, NULL, NULL, CV_MOP_OPEN, 1);	// Removes Salt Noise
		for(int y = 0; y < gray->height; y++){
			uchar* ptr = (uchar*) (gray->imageData + y * gray->widthStep);								//implement inversion and pepper removal in single pass
			for(int x = 0; x < gray->width; x++)
				ptr[x] = 255-ptr[x];							// Pepper becomes Salt
		}
		cvMorphologyEx(gray, gray, NULL, NULL, CV_MOP_OPEN, 1);	// Removes Pepper Noise
		
		//------------------------ Finding Gradient Image ---------------------------------------//
		int **A = (int **) calloc(gray->height, sizeof(int));
		
		IplImage *gradient = cvCreateImage(cvGetSize(gray), 8, 1);
		float min=255, max=0;
		for(int i=1; i<gray->height-1; i++)
		{
			A[i] = (int*) calloc(gray->width, sizeof(int));
			uchar* ptr1 = (uchar*) (gray->imageData + (i-1)*gray->widthStep);
			uchar* ptr2 = (uchar*) (gray->imageData + i*gray->widthStep);
			uchar* ptr3 = (uchar*) (gray->imageData + (i+1)*gray->widthStep);
			uchar* ptr = (uchar*) (gradient->imageData + i*gradient->widthStep);
			for(int j=1; j<gray->width-1; j++)
			{
				A[i][j] = (int)ptr1[j] + (int)ptr2[j-1] + (int)(-4)*ptr2[j] + (int)ptr2[j+1] + (int)ptr3[j];
				if(A[i][j] <= min)
					min = (int)A[i][j];
				if(A[i][j] >= max)
					max = (int)A[i][j];
			}
		}
		//printf("max:%f min:%f\n", max, min);
		int *bins = (int *)calloc(256, sizeof(int));
		for(int i=0; i<256; i++)
			bins[i] = 0;
		for(int i=1; i<gradient->height-1; i++)
		{
			uchar* ptr = (uchar*) (gradient->imageData + i*gradient->widthStep);
			for(int j=1; j<gradient->width-1; j++)
			{
				ptr[j] = (int)(A[i][j]-min)/(max-min)*255;
				bins[ptr[j]]++;	
			}
			free(A[i]);
		}
		free(A);

		//------------ Applying 90 Percentile Threshold to get a Laplacian Edge Mask ------------//
		float sum=0;
		int pth=255;
		float percentile = 10;
		while((pth >= 0)&&(sum < gradient->imageSize*percentile/100))
			sum += bins[pth--];
		//printf("Percentile Threshold:%d\n", pth);

		//--------------- Applying the Laplacian Edge Mask to Original Gray Image ---------------//
		for(int i=0; i<256; i++)
			bins[i] = 0;
		for(int y = 0; y < gray->height; y++){
			uchar* ptr = (uchar*) (gray->imageData + y * gray->widthStep);
			uchar* ptr1 = (uchar*) (gradient->imageData + y * gradient->widthStep);
			for(int x = 0; x < gray->width; x++){
				if(ptr1[x] > pth){
					bins[ptr[x]]++;
				}
			}
		}
		cvReleaseImage(&gradient);
		//cvReleaseImage(&gray);
		
		//------------- Thresholding the Original Gray Image using The Otsu Threshold -----------//
		int threshold = 255-OtsuThreshold(bins, 256);
		free(bins);

		//------------- Filtering points to get better line fittings ----------------------------//
		//img = cvQueryFrame(capture);
		//img = cvLoadImage("../../Data/13 Oct/img4.jpg");
		//IplImage *otsu = cvCreateImage(cvGetSize(img), 8, 1);
		cvCvtColor(img, gray, CV_BGR2GRAY);
		//cvReleaseImage(&img);

		for(int i=0, d=t; i<gray->height; i++)
		{
			int flag = 0;
			uchar* ptr = (uchar*) (gray->imageData + i*gray->widthStep);
			for(int j=0; j<gray->width; j++)
				if(ptr[j] > threshold)
					ptr[j] = 255; 
				else
					ptr[j] = 0;
			for(int x = 0; x < gray->width; x++)
				if(ptr[x]==255)
				{
					for(int i=0; i<d; i++)
						ptr[x+i] = 0;
					ptr[x+d/2] = 255;
					break;
				}
		}

		//---------------------------------- Hough Lines ----------------------------------------//
		CvMemStorage* line_storage = cvCreateMemStorage(0);
		CvSeq* results =  cvHoughLines2(gray, line_storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI/180, 40, 5, 200);
		cvReleaseImage(&gray);
		printf("lines:%d\tt=%d\n", results->total, t);
		
		float angle = 0.0, temp;
		for(int i = 0; i < results->total; i++)
		{
			CvPoint* line = (CvPoint*)cvGetSeqElem(results, i);
			//cvLine(img, line[0], line[1], CV_RGB(255,0,0), 3, CV_AA, 0);
			if(line[0].y > line[1].y)
				temp = atan((line[0].y - line[1].y + 0.0) / (line[0].x - line[1].x));
			else
				temp = atan((line[1].y - line[0].y + 0.0) / (line[1].x - line[0].x));
			if(temp < 0)
				angle += 90 + 180/3.14*temp;
			else
				angle += 180/3.14*temp - 90;
		}
		printf("angle: %f\n", angle = angle/results->total);
		cvReleaseMemStorage(&line_storage);
		//cvNamedWindow("Hough Lines", 0);
		//cvShowImage("Hough Lines", img);
		
		send(FORWARD, angle);
		
		clock_t end=clock();
		time = diffclock(end,begin);
		printf("Time Taken: %f fps\n", 1000/(time));
		
		int c = cvWaitKey(0);
		if(c=='a')
			t++;
		else if(c == 'z')
			t--;
		else if(c == 'x')
		{
			send(BRAKE, 0);
			break;
		}
	}
	int c = cvWaitKey(0);

	return 0;
}

