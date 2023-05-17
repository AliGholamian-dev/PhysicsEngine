#include "CollisionDetectorIF.h"
#include "CollidingBody.h"

std::vector<CollisionResolutionInfo>
CollisionDetectorIF::doNarrowPhaseOnAllPairs(const std::vector<CollidingPair>& possibleCollidingBodies,
                                             const double& deltaTime)
{
    std::vector<CollisionResolutionInfo> collidingBodies;
    collidingBodies.reserve(possibleCollidingBodies.size());

    for(const auto& possibleCollidingPair : possibleCollidingBodies) {
        const auto collisionResolutionInfo = doNarrowPhaseOnEachPair(possibleCollidingPair, deltaTime);
        if(collisionResolutionInfo) {
            collidingBodies.push_back(*collisionResolutionInfo);
        }
    }

    return collidingBodies;
}

std::vector<CollidingPair> CollisionDetectorIF::doBroadPhaseOnAllPairs(const double& deltaTime) {
    std::vector<CollidingPair> possibleCollidingBodies;
    prepareCollidingBodies();
    while(checkForNextCollidingPairAvailable()) {
        auto collidingPair = getNextCollidingPair();
        const auto& firstBodyLayer = collidingPair.first->getCollisionLayer();
        const auto& secondBodyLayer = collidingPair.second->getCollisionLayer();
        const auto& firstBodyLayerMask = collidingPair.first->getLayerMask();
        const auto& secondBodyLayerMask = collidingPair.second->getLayerMask();

        if(firstBodyLayerMask.isLayerMarked(secondBodyLayer) &&
           secondBodyLayerMask.isLayerMarked(firstBodyLayer))
        {
            if(doBroadPhaseOnEachPair(collidingPair.first, collidingPair.second)) {
                possibleCollidingBodies.push_back(collidingPair);
            }
        }
    }
    return possibleCollidingBodies;
}
