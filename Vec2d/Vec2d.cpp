#include "Arduino.h"
#include "Vec2d.h"
#include "string.h"
Vec2d::Vec2d() {
  x = 0;
  y = 0;
}

Vec2d::Vec2d(double x, double y){
  this->x = x;
  this->y = y;
}

String Vec2d::toString()
{
    return "(" + String(x) + ", " + String(y) +")";
}


Vec2d operator-(Vec2d p1, Vec2d p2)
{
    return Vec2d {p1.x - p2.x, p1.y - p2.y };
}

Vec2d operator+(Vec2d p1, Vec2d p2)
{
    return Vec2d {p1.x + p2.x, p1.y + p2.y };
}

Vec2d operator*(Vec2d v, double s)
{
    return Vec2d { v.x * s, v.y * s };
}

Vec2d operator*(double s, Vec2d v)
{
    return Vec2d { v.x * s, v.y * s };
}
bool operator== (Vec2d p1, Vec2d p2)
{
    return(p1.x == p2.x && p1.y==p2.y);
}


double operator*(Vec2d p1, Vec2d p2)
{
    return(p1.x*p2.x+p1.y*p2.y);
}
