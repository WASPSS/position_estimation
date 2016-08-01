#ifndef POSITION_ESTIMATION_H
#define POSITION_ESTIMATION_H

#include <math.h>

struct Position {
    float x;
    float y;
    float z;
};


float SqDistance(Position one, Position two) {
    float ex = one.x - two.x;
    float ey = one.y - two.y;
    float ez = one.z - two.z;
    return ex*ex + ey*ey + ez*ez;
}

Position newPosition(float xx, float yy, float zz) {
    Position ret;
    ret.x = xx;
    ret.y = yy;
    ret.z = zz;
    return ret;
}

#endif // POSITION_ESTIMATION_H
