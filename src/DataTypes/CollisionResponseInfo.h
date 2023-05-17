#pragma once


#include "CollidingPair.h"
#include "Position2D.h"
#include <vector>

struct PenetrationContactPointsPair
{
    Vector2D penetrationVector;
    std::vector<Position2D> contactPoints;
};

struct CollisionResponseInfo {
    CollidingPair collidingPair;
    std::vector<PenetrationContactPointsPair> penetrationContactPointsPairs;
};
