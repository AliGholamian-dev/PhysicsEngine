#include "CollisionResponseIF.h"
#include "CollidingBody.h"


void CollisionResponseIF::respondToAllCollidingBodies(const std::vector<CollisionResponseInfo>& collisionResponseInfos,
                                                      const double& deltaTime)
{
    for(const auto& collisionResponseInfo : collisionResponseInfos) {
        respondToEachCollidingPair(collisionResponseInfo, deltaTime);
    }
}
