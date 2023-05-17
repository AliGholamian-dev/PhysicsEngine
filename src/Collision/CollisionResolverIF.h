#pragma once

#include "CollisionResolutionInfo.h"
#include "CollisionResponseInfo.h"
#include <vector>

class CollisionResolverIF {
    public:
        CollisionResolverIF() = default;
        virtual ~CollisionResolverIF() = default;
        virtual optional<CollisionResponseInfo>
        resolveEachCollidingPair(const CollisionResolutionInfo& collisionInfo, const double& deltaTime) = 0;

        std::vector<CollisionResponseInfo>
        resolveAllCollidingPairs(const std::vector<CollisionResolutionInfo>& collisionResolutionInfos,
                                 const double& deltaTime);
};
