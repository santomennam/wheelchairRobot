#include "Arduino.h"
#include "Vec2d.h"

Vec2d::Vec2d() {
  x = 0;
  y = 0;
}

Vec2d::Vec2d(double x, double y){
  this->x = x;
  this->y = y;
}
