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
#endif
