#include "PolygonHelper.h"
#include "MathUtils.h"
#include "Exceptions.h"
#include "PolygonTriangulation.h"


bool PolygonHelper::checkNumberOfVertices(const std::vector<Position2D>& vertices) {
    return vertices.size() > 2;
}

bool PolygonHelper::checkForUniqueVertices(const std::vector<Position2D>& vertices) {
    int verticesSize = static_cast<int>(vertices.size());
    for(int i = 0; i < verticesSize; i++) {
        for(int j = i + 1; j < verticesSize; j++) {
            const Position2D& currentVertex = vertices[i];
            const Position2D& nextVertex= vertices[j];
            if(MathUtils::checkForEquality(currentVertex.getDistanceTo(nextVertex), 0)) {
                return false;
            }
        }
    }
    return true;
}

bool PolygonHelper::checkForNonCollinearEdges(const std::vector<Position2D>& vertices) {
    int verticesSize = static_cast<int>(vertices.size());
    for(int i = 0; i < verticesSize; i++) {
        const Position2D& currentVertex = vertices[i];
        const Position2D& prevVertex= vertices[MathUtils::wrapIndexAround(i - 1, verticesSize)];
        const Position2D& nextVertex= vertices[MathUtils::wrapIndexAround(i + 1, verticesSize)];

        Vector2D edge1 = prevVertex - currentVertex;
        Vector2D edge2 = nextVertex - currentVertex;
        double dotProduct = edge2.getCrossProductTo(edge1);
        if(MathUtils::checkForEquality(dotProduct, 0)) {
            return false;
        }
    }
    return true;
}

void PolygonHelper::checkPolygonSpecs(const std::vector<Position2D>& vertices) {
    bool inputParametersAreOk = checkNumberOfVertices(vertices);
    inputParametersAreOk = inputParametersAreOk && checkForUniqueVertices(vertices);
    inputParametersAreOk = inputParametersAreOk && checkForNonCollinearEdges(vertices);
    if(!inputParametersAreOk) {
        throw WrongParametersException();
    }
}

std::vector<TriangleInfo> PolygonHelper::getTriangles(const std::vector<Position2D>& vertices) {
    checkPolygonSpecs(vertices);
    PolygonTriangulation polygonTriangulation(vertices);
    return polygonTriangulation.triangulate();
}

double PolygonHelper::getArea(const std::vector<TriangleInfo>& triangles) {
    double area { 0.0 };
    for(const auto& triangle : triangles) {
        area += triangle.area;
    }
    return area;
}

double PolygonHelper::getArea(const std::vector<Position2D>& vertices) {
    checkPolygonSpecs(vertices);
    const auto triangles = getTriangles(vertices);
    return getArea(triangles);
}

double PolygonHelper::getArea(const double& mass, const double& density) {
    if(MathUtils::checkForEquality(mass, 0) ||
       MathUtils::checkForEquality(density, 0) ||
       mass < 0 ||
       density < 0)
    {
        throw WrongParametersException();
    }
    return mass / density;
}

double PolygonHelper::getMass(const double& area, const double& density) {
    if(MathUtils::checkForEquality(area, 0) ||
       MathUtils::checkForEquality(density, 0) ||
       area < 0 ||
       density < 0)
    {
        throw WrongParametersException();
    }
    return area * density;
}

double PolygonHelper::getMass(const std::vector<TriangleInfo>& triangles, const double& density) {
    return getMass(getArea(triangles), density);
}

double PolygonHelper::getMass(const std::vector<Position2D>& vertices, const double& density) {
    checkPolygonSpecs(vertices);
    return getMass(getArea(vertices), density);
}

double PolygonHelper::getDensity(const double& area, const double& mass) {
    if(MathUtils::checkForEquality(area, 0) ||
       MathUtils::checkForEquality(mass, 0) ||
       area < 0 ||
       mass < 0)
    {
        throw WrongParametersException();
    }
    return mass / area;
}

double PolygonHelper::getDensity(const std::vector<TriangleInfo>& triangles, const double& mass) {
    return getDensity(getArea(triangles), mass);
}

double PolygonHelper::getDensity(const std::vector<Position2D>& vertices, const double& mass) {
    checkPolygonSpecs(vertices);
    return getDensity(getArea(vertices), mass);
}

Position2D PolygonHelper::getCentroid(const std::vector<Position2D>& vertices, const std::vector<TriangleInfo>& triangles) {
    Position2D center;
    const Position2D& reference { vertices.front() };
    double area { 0 };

    for(const auto& triangle : triangles) {
        const auto& firstVertex  = vertices[triangle.clockwiseVertexIndexes[0]];
        const auto& secondVertex = vertices[triangle.clockwiseVertexIndexes[1]];
        const auto& thirdVertex  = vertices[triangle.clockwiseVertexIndexes[2]];
        Position2D p1 = firstVertex  - reference;
        Position2D p2 = secondVertex  - reference;
        Position2D p3 = thirdVertex - reference;
        double triangleArea = triangle.area;
        area += triangleArea;
        center += triangleArea * (p1 + p2 + p3) / 3.0;
    }

    center = (center / area) + reference;
    return center;
}

Position2D PolygonHelper::getCentroid(const std::vector<Position2D>& vertices) {
    checkPolygonSpecs(vertices);
    return getCentroid(vertices, getTriangles(vertices));
}

double PolygonHelper::getMomentOfInertiaByRelativeVertices(const std::vector<Position2D>& relativeVerticesToCentroid,
                                                           const double& density)
{
    checkPolygonSpecs(relativeVerticesToCentroid);
    double momentOfInertia { 0 };

    int verticesCount = static_cast<int>(relativeVerticesToCentroid.size());
    for(int i = 0; i < verticesCount; i++) {
        const auto& thisVertex = relativeVerticesToCentroid[i];
        const auto& nextVertex = relativeVerticesToCentroid[MathUtils::wrapIndexAround(i + 1, verticesCount)];
        momentOfInertia += std::abs(nextVertex.getCrossProductTo(thisVertex)) *
                           (thisVertex.getDotProductTo(thisVertex) +
                            nextVertex.getDotProductTo(nextVertex) +
                            nextVertex.getDotProductTo(thisVertex));
    }

    momentOfInertia *= density;
    momentOfInertia /= 12.0;
    return momentOfInertia;
}

double PolygonHelper::getMomentOfInertiaByAbsoluteVertices(const Position2D& centroid,
                                                           const std::vector<Position2D>& absoluteVertices,
                                                           const double& density)
{
    return getMomentOfInertiaByRelativeVertices(getRelativeVertices(centroid, absoluteVertices), density);
}

double PolygonHelper::getMomentOfInertiaByAbsoluteVertices(const std::vector<Position2D>& absoluteVertices,
                                                           const double& density)
{
    return getMomentOfInertiaByRelativeVertices(getRelativeVertices(absoluteVertices), density);
}

std::vector<Position2D> PolygonHelper::getRelativeVertices(const Position2D& centroid,
                                                           const std::vector<Position2D>& absoluteVertices)
{
    std::vector<Position2D> relativeVertices { absoluteVertices };
    for(auto& vertex : relativeVertices) {
        vertex -= centroid;
    }
    return relativeVertices;
}

std::vector<Position2D> PolygonHelper::getRelativeVertices(const std::vector<Position2D>& absoluteVertices) {
    return getRelativeVertices(getCentroid(absoluteVertices), absoluteVertices);
}

std::vector<Position2D> PolygonHelper::getAbsoluteVertices(const Position2D& centroid,
                                                           const std::vector<Position2D>& relativeVertices)
{
    std::vector<Position2D> absoluteVertices { relativeVertices };
    for(auto& vertex : absoluteVertices) {
        vertex += centroid;
    }
    return absoluteVertices;
}
