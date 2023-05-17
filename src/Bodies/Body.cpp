#include "Body.h"
#include "Exceptions.h"
#include "BodyPointerHolderIF.h"
#include "MathUtils.h"
#include "PolygonHelper.h"
#include <numbers>


static constexpr double maxRotation{2 * std::numbers::pi};

Body::Body(ID id,
           const BodyType& bodyType,
           const Position2D& initialCenterPosition,
           const double& currentRotationOfVertices,
           const std::vector<Position2D>& clockwiseVerticesRelativeToCenter,
           const double& radius,
           const std::vector<TriangleInfo>& triangles,
           const wp<BodyPointerHolderIF>& bodyPointerHolder) :
        id(id),
        bodyType(bodyType),
        geometryType(determineGeometryType(clockwiseVerticesRelativeToCenter)),
        centerPosition(initialCenterPosition),
        rotation(currentRotationOfVertices),
        clockwiseVerticesRelativeToCenter(clockwiseVerticesRelativeToCenter),
        radius(radius),
        triangles(triangles),
        bodyPointerHolder(bodyPointerHolder)
{
    checkInputParameters();
    adjustInputParameters();
    registerBody();
}

Body::~Body() {
    removeBody();
}

void Body::registerBody() {
    if(!bodyPointerHolder.expired()) {
        bodyPointerHolder.lock()->registerBody(id, this);
    }
}

void Body::removeBody() {
    if(!bodyPointerHolder.expired()) {
        bodyPointerHolder.lock()->removeBody(id);
    }
}

Body::GeometryType Body::determineGeometryType(const std::vector<Position2D>& vertices) {

    GeometryType determinedGeometryType { GeometryType::convex };

    int verticesSize = static_cast<int>(vertices.size());
    for(int i = 0; i < verticesSize; i++) {
        const Position2D& currentVertex = vertices[i];
        const Position2D& prevVertex= vertices[MathUtils::wrapIndexAround(i - 1, verticesSize)];
        const Position2D& nextVertex= vertices[MathUtils::wrapIndexAround(i + 1, verticesSize)];

        Vector2D edge1 = prevVertex - currentVertex;
        Vector2D edge2 = nextVertex - currentVertex;
        double crossProduct = edge1.getCrossProductTo(edge2);
        if(crossProduct < 0) {
            determinedGeometryType = GeometryType::concave;
            break;
        }
    }
    return determinedGeometryType;
}

bool Body::checkBodyType() const {
    switch (bodyType) {
        case BodyType::polygon :
        case BodyType::circle :
            return true;
        default:
            return false;
    }
}


bool Body::checkNumberOfVertices() const {
    switch (bodyType) {
        case BodyType::circle : {
            return clockwiseVerticesRelativeToCenter.empty();
        }
        case BodyType::polygon : {
            return clockwiseVerticesRelativeToCenter.size() > 2;
        }
        default:
            return false;
    }
}

bool Body::checkForUniqueVertices() const {
    int verticesSize = static_cast<int>(clockwiseVerticesRelativeToCenter.size());
    for(int i = 0; i < verticesSize; i++) {
        for(int j = i + 1; j < verticesSize; j++) {
            const Position2D& currentVertex = clockwiseVerticesRelativeToCenter[i];
            const Position2D& nextVertex= clockwiseVerticesRelativeToCenter[j];
            if(MathUtils::checkForEquality(currentVertex.getDistanceTo(nextVertex), 0)) {
                return false;
            }
        }
    }
    return true;
}

bool Body::checkRadius() const {

    if(radius < 0 ||
       (bodyType == BodyType::circle && MathUtils::checkForEquality(radius, 0))) {
        return false;
    }
    return true;
}

bool Body::checkForNonCollinearEdges() const {
    int verticesSize = static_cast<int>(clockwiseVerticesRelativeToCenter.size());
    for(int i = 0; i < verticesSize; i++) {
        const Position2D& currentVertex = clockwiseVerticesRelativeToCenter[i];
        const Position2D& prevVertex= clockwiseVerticesRelativeToCenter[MathUtils::wrapIndexAround(i - 1, verticesSize)];
        const Position2D& nextVertex= clockwiseVerticesRelativeToCenter[MathUtils::wrapIndexAround(i + 1, verticesSize)];

        Vector2D edge1 = prevVertex - currentVertex;
        Vector2D edge2 = nextVertex - currentVertex;
        double dotProduct = edge2.getCrossProductTo(edge1);
        if(MathUtils::checkForEquality(dotProduct, 0)) {
            return false;
        }
    }
    return true;
}

bool Body::checkNumberOfTriangles() const {
    switch (bodyType) {
        case BodyType::circle : {
            return triangles.empty();
        }
        case BodyType::polygon : {
            return triangles.size() == (clockwiseVerticesRelativeToCenter.size() - 2);
        }
        default:
            return false;
    }
}

void Body::checkInputParameters() const {
    bool inputParametersAreOk = checkBodyType();
    inputParametersAreOk = inputParametersAreOk && checkNumberOfVertices();
    inputParametersAreOk = inputParametersAreOk && checkForUniqueVertices();
    inputParametersAreOk = inputParametersAreOk && checkRadius();
    inputParametersAreOk = inputParametersAreOk && checkForNonCollinearEdges();
    inputParametersAreOk = inputParametersAreOk && checkNumberOfTriangles();
    if(!inputParametersAreOk) {
        throw WrongParametersException();
    }
}

void Body::adjustInputParameters() {
    correctRotation();
}

void Body::setCenterPosition(const Position2D& newPosition) {
    centerPosition = newPosition;
    requestAbsoluteVerticesUpdate();
}

void Body::setRotation(const double& newRotation) {
    const double differenceFromCurrentRotation = newRotation - rotation;
    rotate(differenceFromCurrentRotation);
    requestAbsoluteVerticesUpdate();
}

void Body::changePositionBy(const Vector2D& displacementVector) {
    centerPosition += displacementVector;
    requestAbsoluteVerticesUpdate();
}

void Body::rotate(const double& rotationAngle) {
    for(auto& vertex : clockwiseVerticesRelativeToCenter) {
        vertex.rotate(rotationAngle);
    }
    rotation += rotationAngle;
    correctRotation();
    requestAbsoluteVerticesUpdate();
}

void Body::rotateAroundCustomPoint(const Position2D& point, const double& rotationAngle) {
    Vector2D pointToCenterVector = centerPosition - point;
    pointToCenterVector.rotate(rotationAngle);
    setCenterPosition(pointToCenterVector + point);
    rotate(rotationAngle);
    requestAbsoluteVerticesUpdate();
}

void Body::adjustRotationToUpperBound() {
    while(rotation > maxRotation) {
        rotation -= maxRotation;
    }
}

void Body::adjustRotationToLowerBound() {
    while(rotation < 0) {
        rotation += maxRotation;
    }
}

void Body::correctRotation() {
    adjustRotationToUpperBound();
    adjustRotationToLowerBound();
}

const ID& Body::getBodyID() const {
    return id;
}

const Body::BodyType& Body::getBodyType() const {
    return bodyType;
}

const Body::GeometryType& Body::getGeometryType() const
{
    return geometryType;
}

const Position2D& Body::getCenterPosition() const {
    return centerPosition;
}

const double& Body::getRotation() const {
    return rotation;
}

const std::vector<Position2D>& Body::getRelativeVertices() const {
    return clockwiseVerticesRelativeToCenter;
}

const std::vector<Position2D>& Body::getAbsoluteVertices() const {
    if(absoluteVerticesNeedUpdate) {
        clockwiseAbsoluteVertices = PolygonHelper::getAbsoluteVertices(centerPosition,
                                                                       clockwiseVerticesRelativeToCenter);
        absoluteVerticesNeedUpdate = false;
    }
    return clockwiseAbsoluteVertices;
}

const double& Body::getRadius() const {
    return radius;
}

const std::vector<TriangleInfo>& Body::getTriangles() const {
    return triangles;
}

void Body::requestAbsoluteVerticesUpdate() {
    absoluteVerticesNeedUpdate = true;
}
