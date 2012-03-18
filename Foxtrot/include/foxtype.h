#include <iostream>
#include <string>
#include <cmath>
#include <vector>

#define POINTNUMBERS 4
#define DIRECTIONS 4
#define ANGLEINCREMENT 45
#define ROBOTRADIUS 14.14213562

class Point
{
	public:
		Point(float x, float y);
		float x;
		float y;
		Point operator+ (Point p);
		Point& operator+= (const Point& p);
};

Point::Point(float x, float y)
{
	this->x=x;
	this->y=y;
}

Point Point::operator+ (Point p)
{
	return Point(this->x+p.x, this->y+p.y);
}

Point& Point::operator+=(const Point& p)
{
            x += p.x ;
            y += p.y ;
            return *this ;
}



class Color_rgb
{
	public:
		Color_rgb(char r, char g, char b);
		char r;
		char g;
		char b;
		
};

Color_rgb::Color_rgb(char r, char g, char b)
{
	this->r=r;
	this->g=g;
	this->b=b;
}

