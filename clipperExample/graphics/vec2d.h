#ifndef VEC2D_H
#define VEC2D_H

#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>
#include <string>
#include <complex>

class Vec2d
{
public:
    double x;
    double y;

    constexpr Vec2d() : x{0}, y{0} {}

    template<typename TX, typename TY>
    constexpr Vec2d(TX xvalue, TY yvalue) : x{static_cast<double>(xvalue)}, y{static_cast<double>(yvalue)} {}

    // methods
    double magnitude() const;
    constexpr double magSquared() const { return x*x+y*y; }

    void scale(double s);
    void rotate(double radians);
    void translate(Vec2d offset);

    Vec2d rotated(double radians) const { Vec2d res = *this; res.rotate(radians); return res; }
    Vec2d scaled(double s) const { Vec2d res = *this; res.scale(s); return res; }
    Vec2d translated(Vec2d offset) const { Vec2d res = *this; res.translate(offset); return res; }

    bool equals(const Vec2d& other, double threshold) const;

    Vec2d unit() const { auto m = magnitude(); return { x/m, y/m }; }

    Vec2d operator-() const { return {-x, -y}; }
};

// some handy operators
Vec2d operator+(Vec2d p1, Vec2d p2);
Vec2d operator-(Vec2d p1, Vec2d p2);
Vec2d operator*(Vec2d p1, double s);
Vec2d operator/(Vec2d p1, double s);
Vec2d operator*(double s, Vec2d p1);
//bool  operator== (Vec2d p1, Vec2d p2);

Vec2d& operator+=(Vec2d& v, const Vec2d& other);
Vec2d& operator-=(Vec2d& v, const Vec2d& other);
Vec2d& operator/=(Vec2d& v, double s);
Vec2d& operator*=(Vec2d& v, double s);

#endif // VEC2D_H
