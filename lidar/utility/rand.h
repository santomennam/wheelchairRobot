#ifndef RAND_H
#define RAND_H

#include <random>

class Rand
{
    std::random_device randDevice;
    std::mt19937 mersenneTwister;
public:
    Rand();
    int    randomInt(int minVal, int maxVal);
    double randomDouble(double minVal, double maxVal);
    bool   randomTrue(double pct);
};

#endif // RAND_H
