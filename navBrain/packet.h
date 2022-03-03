#ifndef PACKET_H
#define PACKET_H
#include "segment.h"

class Node;
class packet
{
public:
    double distance = -1;
    Node* node = nullptr;
    segment* seg = nullptr;
    Vec2d closestOnNode;
public:
    packet();
};

#endif // PACKET_H
