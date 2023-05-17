#pragma once


#include "PointerHelper.h"
#include "CollisionDetectorIF.h"
#include "Position2D.h"
#include "MapBasedBodyPointerHolder.h"
class Body;

class SATCollisionDetector final : public CollisionDetectorIF {
    public:
        explicit SATCollisionDetector(const wp<MapBasedBodyPointerHolder>& mapBasedBodyPointerHolder);
        ~SATCollisionDetector() override = default;

        void prepareCollidingBodies() override;
        bool checkForNextCollidingPairAvailable() override;
        CollidingPair getNextCollidingPair() override;
        bool doBroadPhaseOnEachPair(CollidingBody* firstCollidingBody,
                                    CollidingBody* secondCollidingBody) override;

        optional<CollisionResolutionInfo>
        doNarrowPhaseOnEachPair(const CollidingPair& possibleCollidingBodies, const double& deltaTime) override;

    private:
        struct AABB {
            double xMin;
            double xMax;
            double yMin;
            double yMax;
        };
        struct Projection {
            double min;
            double max;
        };
        [[nodiscard]] static AABB computePolygonAABB(const Body* body) ;
        [[nodiscard]] static AABB computeCircleAABB(const Body* body) ;
        [[nodiscard]] static AABB computeAABB(const Body* body) ;
        [[nodiscard]] static bool checkForIntersectingAABBs(const AABB& first, const AABB& second) ;
        [[nodiscard]] static bool checkForAABBIntersection(const Body* firstBody, const Body* secondBody) ;
        [[nodiscard]] static bool checkForTwoCirclesIntersection(const Body* firstBody,
                                                                 const Body* secondBody,
                                                                 CollisionResolutionInfo& collisionInfo) ;
        static std::vector<Position2D> getConvexBodyAbsoluteVertices(const Body* convexBody, const Body* collidingWithBody);
        [[nodiscard]] static Vector2D getPerpendicularAxisOfEdge(const Vector2D& edge) ;
        [[nodiscard]] static Projection projectBodyToAxis(const Body* body, const Vector2D& axis);
        [[nodiscard]] bool checkProjectionIntersectionAgainstAllAxes(const std::vector<Position2D>& verticesOfAxes,
                                                                     const Body* firstBody,
                                                                     const Body* secondBody);
        std::vector<const Body*> getConvexShapes(const Body* body);
        [[nodiscard]] bool checkForCollisionUsingSAT(const Body* firstBody,
                                                     const Body* secondBody,
                                                     CollisionResolutionInfo& collisionInfo);
        void clearTemporaryBodiesVector();

        const wp<MapBasedBodyPointerHolder> mapBasedBodyPointerHolder;
        sp<MapBasedBodyPointerHolder> holderMap;
        double penetrationDepth { 0 };
        Vector2D penetrationDirection;
        std::vector<sp<Body>> temporaryBodiesHolder;

		MapBasedBodyPointerHolder::CollidingBodiesMap::iterator collidingBodiesFirstIt;
        MapBasedBodyPointerHolder::CollidingBodiesMap::iterator collidingBodiesSecondIt;
		MapBasedBodyPointerHolder::CollidingBodiesMap::iterator beforeEndIt;
};
