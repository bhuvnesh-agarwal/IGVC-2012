#include "stdafx.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <cv.h>
#include <highgui.h>
#include <math.h>
//#include "tserial.h"
#include "id.h"
typedef struct location
{
	double x, y;	// Coordinates
	double theta;	// Angle w.r.t horizontal (x-axis)
}location;
typedef struct elem
{
	location l, parent;		// Location of this Element
	double g, h;	// Costs
	int id;			// The identity of this point w.r.t its parent
}elem;
typedef struct list
{
	elem p;
	list *next;
}list;
elem searchforcoor(list *cl,double x,double y){
	list *t = cl;
	while(t)
	{
		//cvLine(img,cvPoint(t->p.parent.x*40,500-(t->p.parent.y*40)),cvPoint(t->p.parent.x*40+2,500-(t->p.parent.y*40)-2),CV_RGB(255,255,0),2,CV_AA,0);
		//printf("(%.3f, %.3f) ", t->p.l.x, t->p.l.y);
		if((t->p.l.x==x)&&(t->p.l.y==y)){
			return t->p;
		}
		t=t->next;
	}
}
list *append(list *l, elem c)
{
	list *node = (list *)malloc(sizeof(list));
	node->p = c;
	node->next = NULL;

	list *t = l;
	if(t == NULL)
		return node;
	while(t->next != NULL)
	{
		if((t->p.l.x == c.l.x) && (t->p.l.y == c.l.y))
		{
			t->p = c;
			return l;
		}
		t = t->next;
	}
	t->next = node;
	return l;
}
bool isNear(location a, location b)
{
	if((pow(a.x - b.x, 2) + pow(a.y - b.y, 2) < 800) && (abs(a.theta - b.theta) < 15))
		return true;
	else
		return false;
}
double radiusOfCurvature(double l, double r)
{
	if(l == r)
		return INFINITY;
	else
	{
		double L = 2;
		return (L/2) * (l+r) / (l-r);
	}
}
double genX(double r, double R)
{
	return r*r / (2*R);
}
double genY(double r, double R)
{
	return r/(2*R)*sqrt(4*R*R - r*r);
}
double genAngle(double r, double R)
{
	double angle = atan((2*R*R - r*r) / (r*sqrt(4*R*R - r*r)))*(180 / 3.14);
	if(angle >= 0)
		return angle;
	else
		return angle + 180;
}
void printPos(elem p)
{
	printf("(%.3f, %.3f, %.3f):(%d)\n", p.l.x, p.l.y, p.l.theta, p.id);
}
elem *neighbours(elem p, elem *np1, int n)
{
	double r = 1;
	double R;

	elem *np = (elem *)malloc(n*sizeof(elem));
	for(int i=0; i<n; i++)
		np[i] = np1[i];
	
	//np[4].l.x = 0;	np[4].l.y = r;	np[4].l.theta = 90;	
	//np[4].g = 10;	np[4].h = INFINITY; np[4].id = _55;
	//for(int i = 0; i < 4; i++)	// Reference Points
	//{
	//	R = radiusOfCurvature(10-(i+1), i+1);
	//	np[i].l.x = genX(r, R);
	//	np[i].l.y = genY(r, R);
	//	np[i].l.theta = genAngle(r, R);
	//	np[i].g = 10;	np[i].h = INFINITY;

	//	np[8-i].l.x = -np[i].l.x;
	//	np[8-i].l.y = np[i].l.y;
	//	np[8-i].l.theta = 180 - np[i].l.theta;
	//	np[8-i].g = np[i].g;	np[8-i].h = INFINITY;
	//}
	/*np[0].id = _91;	np[1].id = _82;	np[2].id = _73;	np[3].id = _64;
	np[8].id = _19;	np[7].id = _28;	np[6].id = _37;	np[5].id = _46;*/

	/*printf("Reference\n");
	for(int i = 0; i < 9; i++)
		printPos(np[i]);*/

	for(int i = 0; i < n; i++)	// Rotation
	{
		double tx, ty;
		tx = np[i].l.x; 
		ty = np[i].l.y;
		np[i].l.x = tx*sin(p.l.theta*(3.14/180)) + ty*cos(p.l.theta*(3.14/180));
		np[i].l.y = -tx*cos(p.l.theta*(3.14/180)) + ty*sin(p.l.theta*(3.14/180));
		np[i].l.theta = np[i].l.theta - (90 - p.l.theta);
	}
	
	/*printf("Rotation\n");
	for(int i = 0; i < 9; i++)
		printPos(np[i]);
*/
	for(int i = 0; i < n; i++)
	{
		np[i].l.x += p.l.x;
		np[i].l.y += p.l.y;
		np[i].parent = p.l;
	}
	//printf("Translation\n");
	/*for(int i = 0; i < n; i++)
		printPos(np[i]);*/
	
	return np;
}
bool walkable(location l)
{
	double x, y, r;
	x = 300, y = 300, r = 50;

	if(pow(l.x - x, 2) + pow(l.y - y, 2) < r*r)
		return false;
	return true;
}

list *update(list *ol, elem e, location t, elem *np, int n)
{
	elem *nextElem = neighbours(e, np, n);
	for(int i = 0; i < n; i++)
	{
		//printPos(nextElem[i]);
		//nextElem[i].h = abs(nextElem[i].l.x - t.x) + abs(nextElem[i].l.y - t.y);	// Manhattan Distance
		nextElem[i].h = sqrt(pow((nextElem[i].l.x - t.x),2)+pow(nextElem[i].l.y - t.y,2));  //euclidean distance
		nextElem[i].g += np->g;
		if(walkable(nextElem[i].l))
			ol = append(ol, nextElem[i]);
	}
	
	return ol;
}
elem findMin(list *l)
{
	double minF = 1000;
	elem min;
	list *t = l;
	while(t != NULL)
	{
		double f = t->p.g + t->p.h;
		//printf("f: %d\n", f);
		if(f < minF)
		{
			minF = f;
			min = t->p;
		}
		t = t->next;
	}
	return min;
}
list *detach(list *l, elem c)
{
	list *t = l;
	if((t == NULL) || (t->next == NULL))
		return NULL;
	if((t->p.l.x == c.l.x) && (t->p.l.y == c.l.y))
	{
		return l->next;
	}

	while(t->next != NULL)
	{
		if((t->next->p.l.x == c.l.x) && (t->next->p.l.y == c.l.y))
			break;
		t = t->next;
	}
	if(t->next != NULL)
		t->next = t->next->next;
	
	return l;
}
void printList(list *l)
{
	list *t = l;
	while(t != NULL)
	{
		printf("(%.3f, %.3f, %.3f) ", t->p.l.x, t->p.l.y, t->p.l.theta);
		t = t->next;
	}
	printf("\n");
}
elem *loadPosData(int n)
{
	float x, y, z, g;
	elem* np = (elem *)malloc(n*sizeof(elem));
	FILE *fp = fopen("n9.txt", "r");
	
	for(int i=0; i<n; i++)
	{
		fscanf(fp, "%f", &x);
		fscanf(fp, "%f", &y);
		fscanf(fp, "%f", &z);
		fscanf(fp, "%f", &g);
		np[i].l.x = x;
		np[i].l.y = y;
		np[i].l.theta = z;
		np[i].g = g;
		np[i].h = INFINITY;
		np[i].id = i+1;
		//printPos(np[i]);
	}
	fclose(fp);
	return np;
}

int _tmain(int argc, _TCHAR* argv[])
{
	location bot, target;
	bot.x = 300;		bot.y = 20;		bot.theta = 90.0;
	target.x = 300;	target.y = 450;	target.theta = 90.000;

	list *ol = NULL, *cl = NULL;
	elem e,vare;
	e.l = bot;	e.g = 0;	e.h = 0;	e.id = UNDEFINED;

	int n = 13;
	elem* np = loadPosData(n);
	
	while(1)
	{
		cl = append(cl, e);
		//printList(cl);
		if(isNear(e.l, target))
			break;
		ol = update(ol, e, target, np, n);
		//printList(ol);
		e = findMin(ol);
		printf("Min: (%.3f, %.3f, %.3f)\n", e.l.x, e.l.y, e.l.theta);
		ol = detach(ol, e);
		//printList(ol);
		//getchar();
	}
	//getchar();
	cvNamedWindow("hello",CV_WINDOW_AUTOSIZE);
	IplImage *img = cvCreateImage(cvSize(500, 500), IPL_DEPTH_8U, 3);
	cvCircle(img, cvPoint(300, 500-300), 45, CV_RGB(0, 15, 200), 1, CV_AA, 0);

	//list *t = cl;
	//while(t)
	//{
	//	cvLine(img,cvPoint(t->p.parent.x*40,500-(t->p.parent.y*40)),cvPoint(t->p.parent.x*40+2,500-(t->p.parent.y*40)-2),CV_RGB(255,255,0),2,CV_AA,0);
	//	//printf("(%.3f, %.3f) ", t->p.l.x, t->p.l.y);
	//	t=t->next;
	//}
	CvPoint a = cvPoint(target.x, 500 - (target.y));
	CvPoint b = cvPoint((target.x + 10*cos(target.theta*(CV_PI/180))), 500 - ((target.y+10*sin(target.theta*(CV_PI/180)))));
	cvLine(img, a, b, CV_RGB(0,255,0), 2, CV_AA, 0);

	a = cvPoint(bot.x, 500 - (bot.y));
	b = cvPoint((bot.x + 10*cos(bot.theta*(CV_PI/180))), 500 - ((bot.y+10*sin(bot.theta*(CV_PI/180)))));
	cvLine(img, a, b, CV_RGB(0,0,255), 2, CV_AA, 0);

	vare = e;
	a = cvPoint(vare.l.x, 500 - (vare.l.y));
	b = cvPoint((vare.l.x + 10*cos(vare.l.theta*(CV_PI/180))), 500 - ((vare.l.y+10*sin(vare.l.theta*(CV_PI/180)))));
	cvLine(img, a, b, CV_RGB(255,0,0), 2, CV_AA, 0);
	
	printf("(%.3f, %.3f, %.3f) : %d\n", vare.l.x, vare.l.y, vare.l.theta, vare.id);
	while(!((abs(vare.l.x-bot.x) < 1.25) && (abs(vare.l.y-bot.y) < 1.25)))
	{
		vare=searchforcoor(cl,vare.parent.x,vare.parent.y);
		if(vare.id != -1)
		{
			printf("(%.3f, %.3f, %.3f) : %d\n", vare.l.x, vare.l.y, vare.l.theta, vare.id);
			a = cvPoint(vare.l.x, 500 - (vare.l.y));
			b = cvPoint((vare.l.x + 10*cos(vare.l.theta*(CV_PI/180))), 500 - ((vare.l.y+10*sin(vare.l.theta*(CV_PI/180)))));
			cvLine(img, a, b, CV_RGB(255,0,0), 2, CV_AA, 0);
		}
	}

	cvShowImage("hello",img);
	cvWaitKey(0);
}