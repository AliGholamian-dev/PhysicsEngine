#include "SATCollisionDetector.h"
#include "Exceptions.h"
#include "CollidingBody.h"
#include "Body.h"
#include "MapBasedBodyPointerHolder.h"
#include "PolygonHelper.h"

SATCollisionDetector::SATCollisionDetector(const wp<MapBasedBodyPointerHolder>& mapBasedBodyPointerHolder) :
        CollisionDetectorIF(),
        mapBasedBodyPointerHolder(mapBasedBodyPointerHolder)
{}

SATCollisionDetector::AABB SATCollisionDetector::computePolygonAABB(const Body* body) {
    AABB aabb{};
    const auto& absoluteVertices = body->getAbsoluteVertices();
    aabb.xMin = absoluteVertices.front().getX();
    aabb.xMax = aabb.xMin;
    aabb.yMin = absoluteVertices.front().getY();
    aabb.yMax = aabb.yMin;
    for(const auto& vertex : absoluteVertices) {
        const double& vertexX = vertex.getX();
        const double& vertexY = vertex.getY();
        if(vertexX > aabb.xMax) {
            aabb.xMax = vertexX;
        }
        else if(vertexX < aabb.xMin) {
            aabb.xMin = vertexX;
        }
        if(vertexY > aabb.yMax) {
            aabb.yMax = vertexY;
        }
        else if(vertexY < aabb.yMin) {
            aabb.yMin = vertexY;
        }
    }
    return aabb;
}

SATCollisionDetector::AABB SATCollisionDetector::computeCircleAABB(const Body* body) {
    AABB aabb{};
    const auto& centerPosition = body->getCenterPosition();
    const auto& radius = body->getRadius();
    aabb.xMin = centerPosition.getX() - radius;
    aabb.xMax = centerPosition.getX() + radius;
    aabb.yMin = centerPosition.getY() - radius;
    aabb.yMax = centerPosition.getY() + radius;
    return aabb;
}

SATCollisionDetector::AABB SATCollisionDetector::computeAABB(const Body* body) {
    switch (body->getBodyType()) {
        case Body::BodyType::polygon: {
            return computePolygonAABB(body);
        }
        case Body::BodyType::circle: {
            return computeCircleAABB(body);
        }
        default:
            throw UnexpectedException();
    }
}

bool SATCollisionDetector::checkForIntersectingAABBs(const SATCollisionDetector::AABB& first,
                                                     const SATCollisionDetector::AABB& second) {
    if(first.xMin > second.xMax ||
       second.xMin > first.xMax ||
       first.yMin > second.yMax ||
       second.yMin > first.yMax)
    {
        return false;
    }
    return true;
}

bool SATCollisionDetector::checkForAABBIntersection(const Body* firstBody, const Body* secondBody) {
    const AABB firstBodyAABB = computeAABB(firstBody);
    const AABB secondBodyAABB = computeAABB(secondBody);
    return checkForIntersectingAABBs(firstBodyAABB, secondBodyAABB);
}

void SATCollisionDetector::prepareCollidingBodies() {
    if(!mapBasedBodyPointerHolder.expired()) {
        holderMap = mapBasedBodyPointerHolder.lock();
        if(holderMap) {
            collidingBodiesFirstIt = holderMap->collidingBodiesMap.begin();
            collidingBodiesSecondIt = collidingBodiesFirstIt;
            collidingBodiesSecondIt++;
			beforeEndIt = holderMap->collidingBodiesMap.end();
			beforeEndIt--;
        }
    }
}

bool SATCollisionDetector::checkForNextCollidingPairAvailable() {
    bool notReachedEnd { false };
    if(holderMap) {
        notReachedEnd = collidingBodiesFirstIt != beforeEndIt;
    }
    if(!notReachedEnd) {
        holderMap.reset();
    }
    return notReachedEnd;
}

CollidingPair SATCollisionDetector::getNextCollidingPair() {
    CollidingPair collidingPair {collidingBodiesFirstIt->second, collidingBodiesSecondIt->second};
    collidingBodiesSecondIt++;
    if (collidingBodiesSecondIt == holderMap->collidingBodiesMap.end()) {
        collidingBodiesFirstIt++;
        collidingBodiesSecondIt = collidingBodiesFirstIt;
        collidingBodiesSecondIt++;
    }
    return collidingPair;
}

bool SATCollisionDetector::doBroadPhaseOnEachPair(CollidingBody* firstCollidingBody,
                                                  CollidingBody* secondCollidingBody)
{
    const auto& firstBody = firstCollidingBody->getBody();
    const auto& secondBody = secondCollidingBody->getBody();
    bool intersect = false;
    if (firstBody && secondBody) {
        intersect = checkForAABBIntersection(firstBody, secondBody);
    }
    return intersect;
}


bool SATCollisionDetector::checkForTwoCirclesIntersection(const Body* firstBody,
                                                          const Body* secondBody,
                                                          CollisionResolutionInfo& collisionInfo)
{
    const auto centerToCenterVector = secondBody->getCenterPosition() - firstBody->getCenterPosition();
    const double sumOfRadii = firstBody->getRadius() + secondBody->getRadius();
    const double centersDistance = centerToCenterVector.getMagnitude();
    if(centersDistance <= sumOfRadii) {
        double depth = sumOfRadii - centersDistance;
        Vector2D centerToCenterNormalizedVector = centerToCenterVector.getNormalizedVector();
        PenetrationInfo penetrationInfo;
        penetrationInfo.penetrationVector = centerToCenterNormalizedVector * depth;
        collisionInfo.penetrationInfos.push_back(penetrationInfo);
        return true;
    }
    return false;
}

std::vector<Position2D> SATCollisionDetector::getConvexBodyAbsoluteVertices(const Body* convexBody,
                                                                            const Body* collidingWithBody)
{
    std::vector<Position2D> absoluteVertices = convexBody->getAbsoluteVertices();

    if (convexBody->getBodyType() == Body::BodyType::circle && collidingWithBody->getBodyType() == Body::BodyType::polygon) {
        absoluteVertices.push_back(convexBody->getCenterPosition());
        Position2D closestVertexToCircleCenter;
        double minDistance = std::numeric_limits<double>::max();
        for(const auto& vertex : collidingWithBody->getAbsoluteVertices()) {
            double distance = vertex.getDistanceTo(convexBody->getCenterPosition());
            if(distance < minDistance) {
                minDistance = distance;
                closestVertexToCircleCenter = vertex;
            }
        }
        absoluteVertices.push_back(closestVertexToCircleCenter);
        for(auto& vertex : absoluteVertices) {
            vertex = getPerpendicularAxisOfEdge(vertex);
        }
    }
    return absoluteVertices;
}

Vector2D SATCollisionDetector::getPerpendicularAxisOfEdge(const Vector2D& edge) {
    return {-edge.getY(), edge.getX()};
}

SATCollisionDetector::Projection SATCollisionDetector::projectBodyToAxis(const Body* body, const Vector2D& axis)
{
    Projection projection{};

    switch (body->getBodyType()) {
        case Body::BodyType::polygon: {
            const auto& vertices = body->getAbsoluteVertices();
            double firstProjection = vertices.front().getDotProductTo(axis);
            projection.min = firstProjection;
            projection.max = firstProjection;
            for(const auto& vertex : vertices) {
                double projectedPoint = vertex.getDotProductTo(axis);
                if(projectedPoint < projection.min) {
                    projection.min = projectedPoint;
                }
                if(projectedPoint > projection.max) {
                    projection.max = projectedPoint;
                }
            }
            break;
        }
        case Body::BodyType::circle: {
            const auto& radius = body->getRadius();
            const auto radiusDirectionalVector = axis * radius;
            const auto circleHighMargin =  body->getCenterPosition() + radiusDirectionalVector;
            const auto circleLowMargin = body->getCenterPosition() - radiusDirectionalVector;
            projection.max = circleHighMargin.getDotProductTo(axis);
            projection.min = circleLowMargin.getDotProductTo(axis);
            break;
        }
    }

    if(projection.min > projection.max) {
        std::swap(projection.min, projection.max);
    }

    return projection;
}

bool SATCollisionDetector::checkProjectionIntersectionAgainstAllAxes(const std::vector<Position2D>& verticesOfAxes,
                                                                     const Body* firstBody,
                                                                     const Body* secondBody)
{
    for (int i = 0; i < verticesOfAxes.size(); i++) {
        const auto& firstVertex = verticesOfAxes[i];
        const auto& secondVertex = verticesOfAxes[(i + 1) % verticesOfAxes.size()];
        Vector2D edge = secondVertex - firstVertex;
        Vector2D axis = getPerpendicularAxisOfEdge(edge);
        axis = axis.getNormalizedVector();

        const auto firstBodyProjection = projectBodyToAxis(firstBody, axis);
        const auto secondBodyProjection = projectBodyToAxis(secondBody, axis);
        if(firstBodyProjection.min > secondBodyProjection.max ||
           secondBodyProjection.min > firstBodyProjection.max)
        {
            return false;
        }

        double currentPenetrationDepth = std::min(secondBodyProjection.max - firstBodyProjection.min,
                                                  firstBodyProjection.max - secondBodyProjection.min);
        if(currentPenetrationDepth < penetrationDepth) {
            penetrationDepth = currentPenetrationDepth;
            penetrationDirection = axis;
        }
    }
    return true;
}

std::vector<const Body*> SATCollisionDetector::getConvexShapes(const Body*body) {
    std::vector<const Body*> convexPolygons;
    const auto& firstBodyGeometryType = body->getGeometryType();

    if(firstBodyGeometryType == Body::GeometryType::convex) {
        convexPolygons.push_back(body);
    }
    else {
        const auto& concaveBodyTriangles = body->getTriangles();
        convexPolygons.reserve(concaveBodyTriangles.size());
        for(const auto& triangle : concaveBodyTriangles) {
            const auto& concaveVertices = body->getAbsoluteVertices();
            const auto& firstVertexOfTriangle = concaveVertices[triangle.clockwiseVertexIndexes[0]];
            const auto& secondVertexOfTriangle = concaveVertices[triangle.clockwiseVertexIndexes[1]];
            const auto& thirdVertexOfTriangle = concaveVertices[triangle.clockwiseVertexIndexes[2]];
            std::vector<Position2D> triangleVertices {firstVertexOfTriangle, secondVertexOfTriangle, thirdVertexOfTriangle};
            const Position2D centerPosition = PolygonHelper::getCentroid(triangleVertices);
            triangleVertices = PolygonHelper::getRelativeVertices(centerPosition, triangleVertices);
            std::vector<TriangleInfo> triangles {triangle};
            sp<Body> triangleBody;
            triangleBody = std::make_shared<Body>(body->getBodyID(),
                                                  body->getBodyType(),
                                                  centerPosition,
                                                  body->getRotation(),
                                                  triangleVertices,
                                                  body->getRadius(),
                                                  triangles,
                                                  sp<BodyPointerHolderIF>(nullptr));
            temporaryBodiesHolder.push_back(triangleBody);
            convexPolygons.push_back(triangleBody.get());
        }
    }

    return convexPolygons;
}

bool SATCollisionDetector::checkForCollisionUsingSAT(const Body* firstBody,
                                                     const Body* secondBody,
                                                     CollisionResolutionInfo& collisionInfo)
{
    bool collide { false };
    std::vector<const Body*> firstBodyConvexShapes= getConvexShapes(firstBody);
    std::vector<const Body*> secondBodyConvexShapes= getConvexShapes(secondBody);

    int index1 = 0;
    for (const auto& firstConvexShape : firstBodyConvexShapes) {
        int index2 = 0;
        for (const auto& secondConvexShape : secondBodyConvexShapes) {
            penetrationDepth = std::numeric_limits<double>::max();
            penetrationDirection = Vector2D();

            const auto firstBodyAbsoluteVertices = getConvexBodyAbsoluteVertices(firstConvexShape, secondConvexShape);
            const auto secondBodyAbsoluteVertices = getConvexBodyAbsoluteVertices(secondConvexShape, firstConvexShape);
            if(checkProjectionIntersectionAgainstAllAxes(firstBodyAbsoluteVertices, firstConvexShape, secondConvexShape) &&
               checkProjectionIntersectionAgainstAllAxes(secondBodyAbsoluteVertices, firstConvexShape, secondConvexShape)) {

                PenetrationInfo penetrationInfo;
                if(firstBody->getGeometryType() == Body::GeometryType::concave) {
                    penetrationInfo.firstBodyTriangleIndex = index1;
                }
                if(secondBody->getGeometryType() == Body::GeometryType::concave) {
                    penetrationInfo.secondBodyTriangleIndex = index2;
                }
                penetrationInfo.penetrationVector = penetrationDepth * penetrationDirection;
                Position2D centerToCenterVector = secondConvexShape->getCenterPosition() - firstConvexShape->getCenterPosition();
                if(centerToCenterVector.getDotProductTo(penetrationInfo.penetrationVector) < 0) {
                    penetrationInfo.penetrationVector *= -1;
                }

                collisionInfo.penetrationInfos.push_back(penetrationInfo);
                collide = true;
            }
            index2++;
        }
        index1++;
    }

    return collide;
}

void SATCollisionDetector::clearTemporaryBodiesVector() {
    temporaryBodiesHolder.clear();
}

optional<CollisionResolutionInfo>
SATCollisionDetector::doNarrowPhaseOnEachPair(const CollidingPair& possibleCollidingBodies, const double& deltaTime) {
    optional<CollisionResolutionInfo> collisionResolutionInfo;

    const auto& firstCollidingBody = possibleCollidingBodies.first;
    const auto& secondCollidingBody = possibleCollidingBodies.second;
    const auto& firstBody = firstCollidingBody->getBody();
    const auto& secondBody = secondCollidingBody->getBody();

    bool collide;

    CollisionResolutionInfo collisionInfo;
    collisionInfo.collidingPair.first = firstCollidingBody;
    collisionInfo.collidingPair.second = secondCollidingBody;

    if(firstBody->getBodyType() == Body::BodyType::circle &&
       secondBody->getBodyType() == Body::BodyType::circle)
    {
        collide = checkForTwoCirclesIntersection(firstBody, secondBody, collisionInfo);
    }
    else {
        collide = checkForCollisionUsingSAT(firstBody, secondBody, collisionInfo);
    }

    if(collide) {
        collisionResolutionInfo = collisionInfo;
    }

    clearTemporaryBodiesVector();
    return collisionResolutionInfo;
}
