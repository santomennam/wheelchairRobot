#define _USE_MATH_DEFINES
#include "vec2d.h"
#include <cmath>
#include <algorithm>
#include <limits>
#include <iostream>

using namespace std;
double crossProduct(Vec2d a, Vec2d b)
{
    return a.x*b.y-b.x*a.y;
}

double Vec2d::magnitude() const
{
    return sqrt(x*x + y*y);
}

void Vec2d::scale(double s)
{
    x *= s;
    y *= s;
}

void Vec2d::rotate(double radians)
{
    *this = { x * cos(radians) - y * sin(radians), x * sin(radians) + y * cos(radians) };
}

void Vec2d::translate(Vec2d offset)
{
    x += offset.x;
    y += offset.y;
}

string Vec2d::toString()
{
    return "(" + to_string(x) + ", " + to_string(y) +")";
}

std::ostream& operator<<(std::ostream& os, const Vec2d& vec)
{
    os << "("<<vec.x<<", "<<vec.y<<")";
    return os;
}

bool Vec2d::equals(const Vec2d& other, double threshold) const
{
    return fabs(x - other.x) <= threshold && fabs(y - other.y) <= threshold;
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
