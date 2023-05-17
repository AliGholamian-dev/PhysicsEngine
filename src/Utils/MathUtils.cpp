#include "MathUtils.h"
#include <cmath>


bool MathUtils::checkForEquality(const double& first, const double& second, const double& userEpsilon) {
    return std::abs(second - first) < userEpsilon;
}

bool MathUtils::checkForEquality(const Position2D& first, const Position2D& second, const double& userEpsilon) {
    return checkForEquality(first.getX(), second.getX(), userEpsilon) &&
           checkForEquality(first.getY(), second.getY(), userEpsilon);
}

int MathUtils::wrapIndexAround(int index, int size) {
    index = index % size;
    if(index < 0) {
        index += size;
    }
    return index;
}

double MathUtils::solvePythagoreanEquation(const double& a, const double& b) {
    return std::sqrt((a * a) + (b * b));
}


double MathUtils::getPointToSegmentDistanceSquared(const Position2D& point,
                                                   const Position2D& firstVertex,
                                                   const Position2D& secondVertex,
                                                   Position2D& closestPoint)
{
    const Vector2D edge { secondVertex - firstVertex };
    const Vector2D centerVector { point - firstVertex };
    const auto projection = centerVector.getDotProductTo(edge);
    const auto edgeLenSquared = edge.getSquaredMagnitude();

    if(projection >= edgeLenSquared) {
        closestPoint = secondVertex;
    }
    else if(projection < 0) {
        closestPoint = firstVertex;
    }
    else {
        closestPoint = firstVertex + ((projection / edgeLenSquared) * edge);
    }

    return Vector2D(point - closestPoint).getSquaredMagnitude();
}

double MathUtils::getPointToSegmentDistance(const Position2D& point,
                                            const Position2D& firstVertex,
                                            const Position2D& secondVertex,
                                            Position2D& closestPoint)
{
    return std::sqrt(getPointToSegmentDistanceSquared(point, firstVertex, secondVertex, closestPoint));
}
