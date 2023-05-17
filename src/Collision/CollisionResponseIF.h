#pragma once


#include "CollisionResponseInfo.h"
class CollisionResponseIF {
    public:
        CollisionResponseIF() = default;
        virtual ~CollisionResponseIF() = default;
        virtual void respondToEachCollidingPair(const CollisionResponseInfo& collisionResponseInfo,
                                                const double& deltaTime) = 0;
        void respondToAllCollidingBodies(const std::vector<CollisionResponseInfo>& collisionResponseInfos,
                                         const double& deltaTime);
};
