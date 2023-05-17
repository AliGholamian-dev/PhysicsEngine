#pragma once


#include "PointerHelper.h"
#include "CollidingPair.h"
#include "Vector2D.h"
#include <vector>

struct PenetrationInfo
{
    optional<int> firstBodyTriangleIndex;
    optional<int> secondBodyTriangleIndex;
    Vector2D penetrationVector;
};

struct CollisionResolutionInfo {
    CollidingPair collidingPair;
    std::vector<PenetrationInfo> penetrationInfos;
};
