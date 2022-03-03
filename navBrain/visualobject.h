#ifndef VISUALOBJECT_H
#define VISUALOBJECT_H
#include "node.h"
#include "graphics.h"
class visualObject
{
public:
    Node* node;
    Vec2d pos;
public:
    visualObject();
};

#endif // VISUALOBJECT_H
