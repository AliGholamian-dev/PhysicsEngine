#include "MassBasedCollisionResolver.h"
#include "MathUtils.h"
#include "CollidingBody.h"
#include "RigidBody.h"
#include "Body.h"
#include "PolygonHelper.h"

//--------------------------------------------------------------------------------------------
///TODO: maybe it is better to move all bodies apart and then calculate contact points
/// (for this you should switch to apply force instead of changing velocity directly)
///TODO: CalculateTime of impact and then calculate impulse and movement of impulse force based on that(also rotation)
///TODO: maybe it is better to convert back to previous state but you should have the time of impact then you can apply force instead of changing velocity
//    firstRigidBody->addForce(-1 * impulse);
//    secondRigidBody->addForce(impulse);
//--------------------------------------------------------------------------------------------

MassBasedCollisionResolver::MassBasedCollisionResolver() : CollisionResolverIF() {}

void MassBasedCollisionResolver::moveBodiesApart(const CollisionResolutionInfo& collisionInfo) {
    const auto& firstCollidingBody = collisionInfo.collidingPair.first;
    const auto& secondCollidingBody = collisionInfo.collidingPair.second;
    const auto& firstRigidBody = firstCollidingBody->getRigidBody();
    const auto& secondRigidBody = secondCollidingBody->getRigidBody();
    const auto& firstBody = firstCollidingBody->getBody();
    const auto& secondBody = secondCollidingBody->getBody();

    const auto& firstBodyInverseMass = firstRigidBody->getInverseRigidInfo().inverseMass;
    const auto& secondBodyInverseMass = secondRigidBody->getInverseRigidInfo().inverseMass;
    auto sumInverseMass =  firstBodyInverseMass + secondBodyInverseMass;
    for(const auto& penetrationInfo: collisionInfo.penetrationInfos) {
        auto firstBodyPenetration = (firstBodyInverseMass / sumInverseMass) * penetrationInfo.penetrationVector;
        auto secondBodyPenetration = (secondBodyInverseMass / sumInverseMass) * penetrationInfo.penetrationVector;
        firstBody->changePositionBy(-1 * firstBodyPenetration);
        secondBody->changePositionBy(secondBodyPenetration);
    }
}

Position2D MassBasedCollisionResolver::findCircleCircleContactPoint(const Position2D& firstCenterPosition,
                                                                    const Position2D& secondCenterPosition,
                                                                    const double& firstBodyRadius)
{
    Vector2D centerToCenterNormalizedVector {secondCenterPosition - firstCenterPosition};
    centerToCenterNormalizedVector = centerToCenterNormalizedVector.getNormalizedVector();
    Position2D contactPoint = firstCenterPosition + centerToCenterNormalizedVector * firstBodyRadius;
    return contactPoint;
}

Position2D MassBasedCollisionResolver::findCirclePolygonContactPoint(const Position2D& circleCenter,
                                                                     const std::vector<Position2D>& vertices)
{
    Position2D contactPoint;

    double minDistanceSquared { std::numeric_limits<double>::max() };
    int verticesSize = static_cast<int>(vertices.size());
    for(int i = 0; i < verticesSize; i++) {
        const auto& firstVertex = vertices[i];
        const auto& secondVertex = vertices[MathUtils::wrapIndexAround(i + 1, verticesSize)];
        Position2D closestPoint;
        const double pointToLineDistanceSquared { MathUtils::getPointToSegmentDistanceSquared(circleCenter,
                                                                                              firstVertex,
                                                                                              secondVertex,
                                                                                              closestPoint) };
        if(pointToLineDistanceSquared < minDistanceSquared) {
            minDistanceSquared = pointToLineDistanceSquared;
            contactPoint = closestPoint;
        }
    }

    return contactPoint;
}

std::vector<Position2D>
MassBasedCollisionResolver::findPolygonPolygonContactPoints(const std::vector<Position2D>& firstPolygonVertices,
                                                            const std::vector<Position2D>& secondPolygonVertices)
{
    Position2D contactPoint1;
    Position2D contactPoint2;
    int contactCount { 0 };
    double minDistanceSquared { std::numeric_limits<double>::max() };

    for(const auto& vertex : firstPolygonVertices) {
        int verticesSize = static_cast<int>(secondPolygonVertices.size());
        for(int i = 0; i < verticesSize; i++) {
            const auto& firstVertex = secondPolygonVertices[i];
            const auto& secondVertex = secondPolygonVertices[MathUtils::wrapIndexAround(i + 1, verticesSize)];
            Position2D closestPoint;
            const double pointToLineDistanceSquared { MathUtils::getPointToSegmentDistanceSquared(vertex,
                                                                                                  firstVertex,
                                                                                                  secondVertex,
                                                                                                  closestPoint) };
            if(MathUtils::checkForEquality(pointToLineDistanceSquared,
                                           minDistanceSquared,
                                           halfOfMillimeter))
            {
                if(!MathUtils::checkForEquality(closestPoint,
                                                contactPoint1,
                                                halfOfMillimeter)) {
                    minDistanceSquared = pointToLineDistanceSquared;
                    contactPoint2 = closestPoint;
                    contactCount = 2;
                }
            }
            else if(pointToLineDistanceSquared < minDistanceSquared) {
                minDistanceSquared = pointToLineDistanceSquared;
                contactPoint1 = closestPoint;
                contactCount = 1;
            }
        }
    }

    for(const auto& vertex : secondPolygonVertices) {
        int verticesSize = static_cast<int>(firstPolygonVertices.size());
        for(int i = 0; i < verticesSize; i++) {
            const auto& firstVertex = firstPolygonVertices[i];
            const auto& secondVertex = firstPolygonVertices[MathUtils::wrapIndexAround(i + 1, verticesSize)];
            Position2D closestPoint;
            const double pointToLineDistanceSquared { MathUtils::getPointToSegmentDistanceSquared(vertex,
                                                                                                  firstVertex,
                                                                                                  secondVertex,
                                                                                                  closestPoint) };
            if(MathUtils::checkForEquality(pointToLineDistanceSquared,
                                           minDistanceSquared,
                                           halfOfMillimeter))
            {
                if(!MathUtils::checkForEquality(closestPoint,
                                                contactPoint1,
                                                halfOfMillimeter)) {
                    minDistanceSquared = pointToLineDistanceSquared;
                    contactPoint2 = closestPoint;
                    contactCount = 2;
                }
            }
            else if(pointToLineDistanceSquared < minDistanceSquared) {
                minDistanceSquared = pointToLineDistanceSquared;
                contactPoint1 = closestPoint;
                contactCount = 1;
            }
        }
    }

    if(contactCount == 1) {
        return { contactPoint1 };
    }
    else if(contactCount == 2) {
        return {contactPoint1,  contactPoint2};
    }
    return {};
}

std::vector<Position2D> MassBasedCollisionResolver::getContactPoints(const Body* firstBody, const Body* secondBody)
{
    std::vector<Position2D> contactPoints;
    const auto& firstBodyType = firstBody->getBodyType();
    const auto& secondBodyType = secondBody->getBodyType();

    if(firstBodyType == Body::BodyType::circle && secondBodyType == Body::BodyType::circle) {
        contactPoints.push_back(findCircleCircleContactPoint(firstBody->getCenterPosition(),
                                                             secondBody->getCenterPosition(),
                                                             firstBody->getRadius()));
    }
    else if((firstBodyType == Body::BodyType::circle && secondBodyType == Body::BodyType::polygon) ||
            (firstBodyType == Body::BodyType::polygon && secondBodyType == Body::BodyType::circle))
    {
        const Body* circle;
        const Body* polygon;

        if(firstBodyType == Body::BodyType::circle) {
            circle = firstBody;
            polygon = secondBody;
        }
        else {
            circle = secondBody;
            polygon = firstBody;
        }
        const auto& circleCenter = circle->getCenterPosition();
        contactPoints.push_back(findCirclePolygonContactPoint(circleCenter, polygon->getAbsoluteVertices()));
    }
    else {
        contactPoints = findPolygonPolygonContactPoints(firstBody->getAbsoluteVertices(),
                                                        secondBody->getAbsoluteVertices());
    }

    return contactPoints;
}

void
MassBasedCollisionResolver::prunePenetrationContactPointsPair(std::vector<PenetrationContactPointsPair>& penetrationContactPointsPair)
{
    for(auto it1 = penetrationContactPointsPair.begin(); it1 != penetrationContactPointsPair.end(); it1++) {
		const auto& currentPair = *it1;

        for(const auto& currentContactPoint : currentPair.contactPoints) {
            for(auto it2 = penetrationContactPointsPair.begin(); it2 != penetrationContactPointsPair.end();) {
                if(it1 != it2) {
					auto& toBeCheckedAgainstPair = *it2;
					for(auto it3 = toBeCheckedAgainstPair.contactPoints.begin(); it3 != toBeCheckedAgainstPair.contactPoints.end();) {
                        const auto& toBeCheckedContactPoint = *it3;
                        if(MathUtils::checkForEquality(currentContactPoint, toBeCheckedContactPoint, halfOfMillimeter)) {
                            it3 = toBeCheckedAgainstPair.contactPoints.erase(it3);
                        }
                        else {
                            it3++;
                        }
                    }
                    if(toBeCheckedAgainstPair.contactPoints.empty()) {
                        it2 = penetrationContactPointsPair.erase(it2);
                    }
                    else {
                        it2++;
                    }
                }
                else{
                    it2++;
                }
            }
        }

    }
}

optional<CollisionResponseInfo>
MassBasedCollisionResolver::resolveEachCollidingPair(const CollisionResolutionInfo& collisionInfo,
                                                     const double& deltaTime)
{
    optional<CollisionResponseInfo> collisionResponseInfo;

    const auto& firstCollidingBody = collisionInfo.collidingPair.first;
    const auto& secondCollidingBody = collisionInfo.collidingPair.second;
    const auto& firstRigidBody = firstCollidingBody->getRigidBody();
    const auto& secondRigidBody = secondCollidingBody->getRigidBody();
    if(firstRigidBody && secondRigidBody) {
        if(firstRigidBody->getRigidBodyType() != RigidBody::RigidBodyType::staticBody ||
           secondRigidBody->getRigidBodyType() != RigidBody::RigidBodyType::staticBody)
        {
            collisionResponseInfo = CollisionResponseInfo();
            collisionResponseInfo->collidingPair = collisionInfo.collidingPair;
            collisionResponseInfo->penetrationContactPointsPairs.reserve(collisionInfo.penetrationInfos.size() * 2);
            const auto& firstBody = firstCollidingBody->getBody();
            const auto& secondBody = secondCollidingBody->getBody();
            if(firstBody && secondBody) {
                moveBodiesApart(collisionInfo);
                std::vector<Position2D> firstBodySelectedVertices = firstBody->getAbsoluteVertices();
                std::vector<Position2D> secondBodySelectedVertices = secondBody->getAbsoluteVertices();
                const Body* body1 { firstBody };
                const Body* body2 { secondBody };
                sp<Body> triangleBody1;
                sp<Body> triangleBody2;

                for(const auto& penetrationInfo: collisionInfo.penetrationInfos) {
                    PenetrationContactPointsPair penetrationContactPointsPair;
                    penetrationContactPointsPair.penetrationVector = penetrationInfo.penetrationVector;

                    if(penetrationInfo.firstBodyTriangleIndex) {
                        const auto& triangle = firstBody->getTriangles()[*penetrationInfo.firstBodyTriangleIndex];
                        const auto& firstVertexOfTriangle = firstBodySelectedVertices[triangle.clockwiseVertexIndexes[0]];
                        const auto& secondVertexOfTriangle = firstBodySelectedVertices[triangle.clockwiseVertexIndexes[1]];
                        const auto& thirdVertexOfTriangle = firstBodySelectedVertices[triangle.clockwiseVertexIndexes[2]];
                        std::vector<Position2D> triangleVertices {firstVertexOfTriangle, secondVertexOfTriangle, thirdVertexOfTriangle};
                        const Position2D centerPosition = PolygonHelper::getCentroid(triangleVertices);
                        triangleVertices = PolygonHelper::getRelativeVertices(centerPosition, triangleVertices);
                        std::vector<TriangleInfo> triangles {triangle};

                        triangleBody1 = std::make_shared<Body>(firstBody->getBodyID(),
                                                               firstBody->getBodyType(),
                                                               centerPosition,
                                                               firstBody->getRotation(),
                                                               triangleVertices,
                                                               firstBody->getRadius(),
                                                               triangles,
                                                               sp<BodyPointerHolderIF>(nullptr));
                        body1 = triangleBody1.get();
                    }

                    if(penetrationInfo.secondBodyTriangleIndex) {
                        const auto& triangle = secondBody->getTriangles()[*penetrationInfo.secondBodyTriangleIndex];
                        const auto& firstVertexOfTriangle = secondBodySelectedVertices[triangle.clockwiseVertexIndexes[0]];
                        const auto& secondVertexOfTriangle = secondBodySelectedVertices[triangle.clockwiseVertexIndexes[1]];
                        const auto& thirdVertexOfTriangle = secondBodySelectedVertices[triangle.clockwiseVertexIndexes[2]];
                        std::vector<Position2D> triangleVertices {firstVertexOfTriangle, secondVertexOfTriangle, thirdVertexOfTriangle};
                        const Position2D centerPosition = PolygonHelper::getCentroid(triangleVertices);
                        triangleVertices = PolygonHelper::getRelativeVertices(centerPosition, triangleVertices);
                        std::vector<TriangleInfo> triangles {triangle};
                        triangleBody2 = std::make_shared<Body>(secondBody->getBodyID(),
                                                               secondBody->getBodyType(),
                                                               centerPosition,
                                                               secondBody->getRotation(),
                                                               triangleVertices,
                                                               secondBody->getRadius(),
                                                               triangles,
                                                               sp<BodyPointerHolderIF>(nullptr));

                        body2 = triangleBody2.get();
                    }
                    auto currentContactPoints = getContactPoints(body1, body2);
                    penetrationContactPointsPair.contactPoints = currentContactPoints;
                    collisionResponseInfo->penetrationContactPointsPairs.push_back(penetrationContactPointsPair);
                }
            }
            prunePenetrationContactPointsPair(collisionResponseInfo->penetrationContactPointsPairs);
        }
    }

    return collisionResponseInfo;
}
