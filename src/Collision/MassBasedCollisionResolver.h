#pragma once


#include "CollisionResolverIF.h"
class Body;


class MassBasedCollisionResolver : public CollisionResolverIF {
    public:
        MassBasedCollisionResolver();
        ~MassBasedCollisionResolver() override = default;

        optional<CollisionResponseInfo>
        resolveEachCollidingPair(const CollisionResolutionInfo& collisionInfo, const double& deltaTime) override;

    private:
        static void moveBodiesApart(const CollisionResolutionInfo& collisionInfo);
        static Position2D findCircleCircleContactPoint(const Position2D& firstCenterPosition,
                                                       const Position2D& secondCenterPosition,
                                                       const double& firstBodyRadius);
        static Position2D findCirclePolygonContactPoint(const Position2D& circleCenter,
                                                        const std::vector<Position2D>& vertices);
        static std::vector<Position2D> findPolygonPolygonContactPoints(const std::vector<Position2D>& firstPolygonVertices,
                                                                       const std::vector<Position2D>& secondPolygonVertices);
        static std::vector<Position2D> getContactPoints(const Body* firstBody, const Body* secondBody);
        static void prunePenetrationContactPointsPair(std::vector<PenetrationContactPointsPair>& penetrationContactPointsPair);
        static constexpr double halfOfMillimeter { 0.0005 };
};
