// contours.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <math.h>
#include <stdlib.h>

#define INFINITE -1

typedef struct position
{
	double x, y, theta;
	double g, h;
}position;

double radiusOfCurvature(double l, double r)
{
	if(l == r)
		return INFINITE;
	else
	{
		double L = 50;
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
void printPos(position p)
{
	printf("x: %.3f\ty: %.3f\ttheta: %.3f\n", p.x, p.y, p.theta);
}
void fprintpos(position *p, int n)
{
	FILE *fp = fopen("data.txt", "w");
	for(int i=0; i<n; i++)
		fprintf(fp, "%.3f %.3f %.3f\n", p[i].x, p[i].y, p[i].theta);
	fclose(fp);
}
void neighbours(position p)
{
	double r = 25;
	double R;

	int n = 19;
	position *np = (position *)malloc(n*sizeof(position));
	
	np[n/2].x = 0;
	np[n/2].y = r;
	np[n/2].theta = 90;
	np[n/2].g = r;
	for(int i = 0; i < n/2; i++)	// Reference Points
	{
		R = radiusOfCurvature((n+1)-(i+1), i+1);
		np[i].x = genX(r, R);
		np[i].y = genY(r, R);
		np[i].theta = genAngle(r, R);
		np[i].g = atan(np[i].y / (R - np[i].x)) * R;

		np[(n-1)-i].x = -np[i].x;
		np[(n-1)-i].y = np[i].y;
		np[(n-1)-i].theta = 180 - np[i].theta;
		np[(n-1)-i].g = np[i].g;
	}
	printf("Reference\n");
	for(int i = 0; i < n; i++)
		printPos(np[i]);

	for(int i = 0; i < n; i++)	// Rotation
	{
		double tx, ty;
		tx = np[i].x; 
		ty = np[i].y;
		np[i].x = tx*sin(p.theta*(3.14/180)) + ty*cos(p.theta*(3.14/180));
		np[i].y = -tx*cos(p.theta*(3.14/180)) + ty*sin(p.theta*(3.14/180));
		np[i].theta = np[i].theta - (90 - p.theta);
	}
	printf("Rotation\n");
	for(int i = 0; i < n; i++)
		printPos(np[i]);

	for(int i = 0; i < n; i++)
	{
		np[i].x += p.x;
		np[i].y += p.y;
	}
	printf("Translation\n");
	for(int i = 0; i < n; i++)
		printPos(np[i]);
	
	fprintpos(np, n);
}
position* read()
{
	char *buf = (char *)malloc(100*sizeof(char));
	FILE *fp = fopen("data.txt", "r");
	fscanf(fp, "%s", buf);
	printf("%s\n", buf);
	return NULL;
}

int _tmain(int argc, _TCHAR* argv[])
{
	position p;
	p.theta = 90;
	p.x = 0;
	p.y = 0;

	neighbours(p);
	//position *np = read();
	
	getchar();	
	return 0;
}

