#pragma once


#include "CollisionResolutionInfo.h"


class CollisionDetectorIF {
    public:
        CollisionDetectorIF() = default;
        virtual ~CollisionDetectorIF() = default;

        virtual void prepareCollidingBodies() = 0;
        virtual bool checkForNextCollidingPairAvailable() = 0;
        virtual CollidingPair getNextCollidingPair() = 0;
        virtual bool doBroadPhaseOnEachPair(CollidingBody* firstCollidingBody,
                                            CollidingBody* secondCollidingBody) = 0;
        virtual optional<CollisionResolutionInfo> doNarrowPhaseOnEachPair(const CollidingPair& possibleCollidingBodies,
                                                                          const double& deltaTime) = 0;

        std::vector<CollidingPair> doBroadPhaseOnAllPairs(const double& deltaTime);
        std::vector<CollisionResolutionInfo>
        doNarrowPhaseOnAllPairs(const std::vector<CollidingPair>& possibleCollidingBodies, const double& deltaTime);
};
