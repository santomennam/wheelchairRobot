#ifndef Vec2d_h
#define Vec2d_h
#include "Arduino.h"
#include "string.h"
class Vec2d{
  public:
   double x;
   double y;
  public:
    String toString();
    Vec2d();
    Vec2d(double x, double y);
};

Vec2d operator+(Vec2d p1, Vec2d p2);
Vec2d operator-(Vec2d p1, Vec2d p2);
Vec2d operator*(Vec2d p1, double s);
Vec2d operator*(double s, Vec2d p1);
double operator*(Vec2d p1, Vec2d p2);
bool  operator== (Vec2d p1, Vec2d p2);

double crossProduct(Vec2d a, Vec2d b);
#endif
