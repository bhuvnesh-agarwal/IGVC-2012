#include "stdafx.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <cv.h>
#include <highgui.h>
#include <math.h>
#include "tserial.h"

#define SAFEZONE_LL (-191)
#define SAFEZONE_RL 159

void findfit(IplImage* img,float *xp,float *yp)
{
	float a1=0,b1=0,c1=0,a2=0,b2=0,c2=0;
	int w=img->height;
	for( int y=10; y<img->height-10; y++ ) 
	{
		uchar* ptr = (uchar*) (img->imageData + y * img->widthStep);
		for( int x=10; x<img->width-10; x++ ) 
		{
			if(ptr[x]>100)
			{
				printf("(%d,%d)",x,y);
				a1+=x*x;
				b1+=x;
				c1+=x*y;
				a2+=x;
				b2+=1;
				c2+=y;
			}
		}
	}
	*xp=(c2*b1-c1*b2)/(a1*b2-a2*b1);
	*yp=(c1*a2-c2*a1)/(a1*b2-a2*b1);
}
IplImage* doCanny(IplImage* in,double lowThresh,double highThresh,double aperture) 
{
	if(in->nChannels != 1)
		return(0); //Canny only handles gray scale images

	IplImage* out = cvCreateImage(cvGetSize( in ),IPL_DEPTH_8U,1);
	cvCanny( in, out, lowThresh, highThresh, aperture );
	return( out );
}
IplImage* simplethreshold(IplImage* img, int threshold) 
{
	for( int y=0; y<img->height; y++ ) 
	{
		uchar* ptr = (uchar*) (img->imageData + y * img->widthStep);
		for( int x=0; x<img->width; x++ ) 
		{
			if(ptr[x+1]>=threshold)
			{
				ptr[x+1] = 255 ;
			}
			else
			{
				ptr[x+1] = 0 ;
			}
		}
	}
	return img;
}
void navCommand(float tangle, float cangle)
{
	printf("tangle-cangle:%f\n", tangle-cangle);
	Tserial* p = new Tserial();
    p->connect("COM2",9600,spNONE);
	if((tangle - cangle < 5) && (tangle - cangle > -5))
	{
		//printf("Foward\n");
		p->sendChar('w');
		p->sendChar('5');
		p->sendChar('5');
	}
	else if(tangle - cangle >= 5)
	{
		//printf("Right\n");
		p->sendChar('d');
	}
	else if(tangle - cangle <= -5)
	{
		//printf("Left\n");
		p->sendChar('a');
	}
	/*if(angle > 1)
	{
		p->sendChar('w');
		p->sendChar('3');
		p->sendChar('7');
	}
	else if (angle < -1)
	{
		p->sendChar('w');
		p->sendChar('7');
		p->sendChar('3');
	}
	else
	{
		p->sendChar('w');
		p->sendChar('5');
		p->sendChar('5');
	}*/

	p->disconnect();
}
IplImage* cutImg(IplImage* img)
{
	for( int y=0; y < 200; y++ ) 
	{
		uchar* ptr = (uchar*) (img->imageData + y * img->widthStep);
		for( int x=0; x<img->width; x++ ) 
		{
			ptr[x] = 0;
		}
	}
	return img;
}
char* nameGen(int idx)
{
	char* name = (char*)malloc(8*sizeof(char));
	int t = idx;
	for(int i=2; i>=0; i--)
	{	
		name[i] = t%10+'0';
		t /= 10;
	}
	name[3] = '.';
	name[4] = 'j';
	name[5] = 'p';
	name[6] = 'g';
	name[7] = '\0';

	return name;
}

void main(int argc, char** argv)
{
	cvNamedWindow("src",0 );
	cvNamedWindow("warp image",0 );
	cvNamedWindow("warp image (grey)",0 );
	cvNamedWindow("Smoothed warped gray",0 );
	cvNamedWindow("threshold image",0 );
	cvNamedWindow("canny",0 );
	cvNamedWindow("final",1 );
		
	CvPoint2D32f srcQuad[4], dstQuad[4];
	CvMat* warp_matrix = cvCreateMat(3,3,CV_32FC1);
	float Z=1;

	dstQuad[0].x = 216; //src Top left
	dstQuad[0].y = 15;
	dstQuad[1].x = 392; //src Top right
	dstQuad[1].y = 6;
	dstQuad[2].x = 12; //src Bottom left
	dstQuad[2].y = 187;
	dstQuad[3].x = 620; //src Bot right
	dstQuad[3].y = 159;

	srcQuad[0].x = 100; //dst Top left
	srcQuad[0].y = 120;
	srcQuad[1].x = 540; //dst Top right
	srcQuad[1].y = 120;
	srcQuad[2].x = 100; //dst Bottom left
	srcQuad[2].y = 360;
	srcQuad[3].x = 540; //dst Bot right
	srcQuad[3].y = 360;

	cvGetPerspectiveTransform(srcQuad, dstQuad,	warp_matrix);
	
	//CvCapture *capture = cvCaptureFromCAM(0);
	/*double fps = cvGetCaptureProperty(capture, CV_CAP_PROP_FPS);
	IplImage* image = cvRetrieveFrame(capture);
	CvSize imgSize;
    imgSize.width = image->width;
    imgSize.height = image->height;
	CvVideoWriter *writer = cvCreateVideoWriter("out.avi", CV_FOURCC('M', 'J', 'P', 'G'), fps, imgSize);*/
	int ik=0;
	while(1)
	{
		//IplImage* img = cvQueryFrame(capture);
		IplImage* img = cvLoadImage( "../../Data/6 Dec/009.jpg", CV_LOAD_IMAGE_COLOR);
		cvShowImage( "src", img );
		//cvWriteFrame(writer, img);
		//cvSaveImage(nameGen(ik++), img, 0);
		
		IplImage* warp_img = cvCloneImage(img);
		CV_MAT_ELEM(*warp_matrix, float, 2, 2) = Z;
		cvWarpPerspective(img, warp_img, warp_matrix, CV_INTER_LINEAR | CV_WARP_INVERSE_MAP | CV_WARP_FILL_OUTLIERS);
		cvShowImage( "warp image", warp_img );

		IplImage* grayimg = cvCreateImage(cvGetSize(warp_img),IPL_DEPTH_8U,1);
		cvCvtColor( warp_img, grayimg, CV_RGB2GRAY );
		cvShowImage( "warp image (grey)", grayimg );
		
		cvSmooth(grayimg, grayimg, CV_GAUSSIAN, 3, 3, 0.0, 0.0);
		cvShowImage( "Smoothed warped gray", grayimg );
		
		IplImage* thresholded_img=simplethreshold(grayimg, 220);
		cvShowImage("threshold image",thresholded_img);

		//grayimg = doCanny( thresholded_img, 50, 100, 3 );
		grayimg = cvCloneImage(thresholded_img);
		cvShowImage("canny",grayimg);

		IplImage* finalimg = cvCreateImage(cvGetSize(grayimg),IPL_DEPTH_8U,3);
		CvMemStorage* line_storage=cvCreateMemStorage(0);

		CvSeq* results =  cvHoughLines2(grayimg,line_storage,CV_HOUGH_PROBABILISTIC,10,CV_PI/180*5,350,100,10);
		double angle = 0.0, temp;
		double lengthSqd, wSum=0;
		double xc = 0, yc = 0;
		for( int i = 0; i < results->total; i++ )
		{
			CvPoint* line = (CvPoint*)cvGetSeqElem(results,i);
			cvLine( finalimg, line[0], line[1], CV_RGB(0,0,255), 1, CV_AA, 0 );
			//lengthSqd = (line[0].x - line[1].x)*(line[0].x - line[1].x) + (line[0].y - line[1].y)*(line[0].y - line[1].y);
			wSum += 1;//lengthSqd;
			if(line[0].y > line[1].y)
				temp = atan((line[0].y - line[1].y + 0.0) / (line[0].x - line[1].x));
			else
				temp = atan((line[1].y - line[0].y + 0.0) / (line[1].x - line[0].x));
			if(temp < 0)
				angle += (90 + 180/3.14*temp)/* * lengthSqd*/;
			else
				angle += (180/3.14*temp - 90)/* * lengthSqd*/;
			xc += line[0].x + line[1].x;
			yc += line[0].y + line[1].y;
		}
		angle=angle/wSum;
		//angle+=10;
		printf("total: %d, angle: % f\n", results->total, angle);

		xc /= 2*results->total;
		yc /= 2*results->total;
		double m = (angle != 0) ? 1/tan(angle*3.14/180) : 100;	// 100 represents a very large slope (near vertical)
		m=-m;

		double x1, y1, x2, y2;	// The Center Line
		y1 = 0;
		y2 = finalimg->height;
		x1 = xc + (y1-yc)/m;
		x2 = xc + (y2-yc)/m; 
		cvLine(finalimg, cvPoint(x1, y1), cvPoint(x2, y2), CV_RGB(0,255,0), 1, CV_AA, 0);
		printf("point: %f\t%f\n", xc, yc);

		double lx=0, ly=0, lm=0, lc=0, rx=0, ry=0, rm=0, rc=0;
		for( int i = 0; i < results->total; i++ )
		{
			CvPoint* line = (CvPoint*)cvGetSeqElem(results,i);
			double xm = (line[0].x + line[1].x)/2.0, ym = (line[0].y + line[1].y)/2.0;
			if(ym - yc - m*(xm - xc) > 0)
			{
				lx += xm;
				ly += ym;
				lm += (line[1].y - line[0].y)/(line[1].x - line[0].x+0.0001);
				lc++;
			}
			else
			{
				rx += xm;
				ry += ym;
				rm += (line[1].y - line[0].y)/(line[1].x - line[0].x+0.0001);
				rc++;
			}
		}

		// The Left Line
		lx /= lc;	ly /= lc;	lm /= lc;
		rx /= rc;	ry /= rc;	rm /= rc;
		printf("lins: %f\t%f\t%f\n", lx, ly, lm);
		printf("lins: %f\t%f\t%f\n", rx, ry, rm);
		y1 = 0;
		y2 = finalimg->height-5;
		x1 = lx + (y1-ly)/lm;
		x2 = lx + (y2-ly)/lm; 
		cvLine(finalimg, cvPoint(x1, y1), cvPoint(x2, y2), CV_RGB(255,255,0), 1, CV_AA, 0);

		// The Right Line
		y1 = 0;
		y2 = finalimg->height-5;
		x1 = rx + (y1-ry)/rm;
		x2 = rx + (y2-ry)/rm; 
		cvLine(finalimg, cvPoint(x1, y1), cvPoint(x2, y2), CV_RGB(0,255,255), 1, CV_AA, 0);

		// The Center Point
		CvPoint vpt = cvPoint(finalimg->width/2, 416);
		printf("center point: %d\t%d\n", vpt.x, vpt.y);
		
		// The Dl and Dr
		int dl = vpt.x - lx + (ly-vpt.y+0.0)/lm;
		int dr = (vpt.y-ry+0.0)/rm + rx - vpt.x;
		printf("dl-dr: %d\n", dl-dr);

		cvShowImage("final",finalimg);

		if(dl-dr < SAFEZONE_LL)	// Assume that the bot lies just on the boundary of the safe zone
		{
			if(angle < -10)
			{
				navCommand(7, angle);
			}
			else
			{
				navCommand(7, angle);
			}
		}	
		else if(dl-dr > SAFEZONE_RL)
		{
			if(angle > 10)
			{
				navCommand(-7, angle);
			}
			else
			{
				navCommand(-7, angle);
			}
		}
		else
		{
			if((angle < 10) && (angle > -10))
			{
				navCommand(angle, angle);
			}
			else
			{
				navCommand(0, angle);
			}
		}

		cvWaitKey(0);
	}
}


