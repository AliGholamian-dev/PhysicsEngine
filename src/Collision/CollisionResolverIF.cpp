#include "CollisionResolverIF.h"


std::vector<CollisionResponseInfo>
CollisionResolverIF::resolveAllCollidingPairs(const std::vector<CollisionResolutionInfo>& collisionResolutionInfos,
                                              const double& deltaTime)
{
    std::vector<CollisionResponseInfo> collisionResponseInfos;
    collisionResponseInfos.reserve(collisionResolutionInfos.size());

    for(const auto& collisionResolutionInfo : collisionResolutionInfos) {
        const auto collisionResponseInfo = resolveEachCollidingPair(collisionResolutionInfo, deltaTime);
        if(collisionResponseInfo) {
            collisionResponseInfos.push_back(*collisionResponseInfo);
        }
    }

    return collisionResponseInfos;
}
