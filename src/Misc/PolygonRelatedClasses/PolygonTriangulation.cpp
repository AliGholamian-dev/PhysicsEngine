#include "PolygonTriangulation.h"
#include "MathUtils.h"
#include <algorithm>

PolygonTriangulation::PolygonTriangulation(const std::vector<Position2D>& vertices) :
        OneTimeUsable(),
        vertices(vertices)
{
    fillIndexes();
}

std::vector<TriangleInfo> PolygonTriangulation::triangulate() {
    ensureOneTimeUsage();
    return getTrianglesByEarCutting();
}

void PolygonTriangulation::fillIndexes() {
    for(int i = 0; i < vertices.size(); i++) {
        indexes.push_back(i);
    }
}

bool PolygonTriangulation::checkForConvexAngle() {
    const Position2D& prevVertex = vertices[prevIndex];
    const Position2D& currentVertex = vertices[currentIndex];
    const Position2D& nextVertex = vertices[nextIndex];

    Vector2D edge1 = prevVertex - currentVertex;
    Vector2D edge2 = nextVertex - currentVertex;
    auto crossProduct = edge1.getCrossProductTo(edge2);

    return crossProduct >= 0;
}

bool PolygonTriangulation::checkForNoVertexInTriangle() {
    const Position2D& prevVertex = vertices[prevIndex];
    const Position2D& currentVertex = vertices[currentIndex];
    const Position2D& nextVertex = vertices[nextIndex];


    Vector2D firstEdge = currentVertex - prevVertex;
    Vector2D secondEdge = nextVertex - currentVertex;
    Vector2D thirdEdge = prevVertex - nextVertex;

    auto checkForContainingVertex = [this, &firstEdge, &secondEdge, &thirdEdge, &prevVertex, &currentVertex, &nextVertex](const int& index) {
        if(index == prevIndex || index == currentIndex || index == nextIndex) {
            return false;
        }
        const Position2D& vertex = vertices[index];
        Vector2D firstVertexToPoint = vertex - prevVertex;
        Vector2D secondVertexToPoint = vertex - currentVertex;
        Vector2D thirdVertexToPoint = vertex - nextVertex;

        bool leftHandSideOfFirstEdge = firstEdge.getCrossProductTo(firstVertexToPoint) > 0;
        bool leftHandSideOfSecondEdge = secondEdge.getCrossProductTo(secondVertexToPoint) > 0;
        bool leftHandSideOfThirdEdge = thirdEdge.getCrossProductTo(thirdVertexToPoint) > 0;

        if(leftHandSideOfFirstEdge  ||  leftHandSideOfSecondEdge || leftHandSideOfThirdEdge) {
            return false;
        }
        return true;
    };

    if(std::none_of(indexes.begin(), indexes.end(), checkForContainingVertex)) {
        return true;
    }
    return false;
}

bool PolygonTriangulation::checkValidEarConditions() {
    if(checkForConvexAngle() && checkForNoVertexInTriangle()) {
        return true;
    }
    return false;
}

std::vector<TriangleInfo> PolygonTriangulation::getTrianglesByEarCutting()
{
    std::vector<TriangleInfo> triangles;

    while(indexes.size() >= 3) {
        int indexesSize = static_cast<int>(indexes.size());
        for(int i = 0; i < indexesSize; i++) {
            currentIndex = indexes[i];
            prevIndex = indexes[MathUtils::wrapIndexAround(i - 1, indexesSize)];
            nextIndex = indexes[MathUtils::wrapIndexAround(i + 1, indexesSize)];

            bool validEar = checkValidEarConditions();
            if(validEar) {
                const Position2D& prevVertex = vertices[prevIndex];
                const Position2D& currentVertex = vertices[currentIndex];
                const Position2D& nextVertex = vertices[nextIndex];

                Vector2D edge1 = prevVertex - currentVertex;
                Vector2D edge2 = nextVertex - currentVertex;
                auto crossProduct = edge1.getCrossProductTo(edge2);

                TriangleInfo triangleInfo{};
                triangleInfo.clockwiseVertexIndexes = { prevIndex, currentIndex, nextIndex };
                triangleInfo.area = crossProduct / 2.0;
                triangles.push_back(triangleInfo);
                indexes.erase(indexes.begin() + i);
                break;
            }
        }
    }

    return triangles;
}
