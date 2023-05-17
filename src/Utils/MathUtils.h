#pragma once

#include <limits>
#include "Position2D.h"

class MathUtils final {
    public:
        static bool checkForEquality(const double& first, const double& second, const double& userEpsilon = epsilon);
        static bool checkForEquality(const Position2D& first, const Position2D& second, const double& userEpsilon = epsilon);
        static int wrapIndexAround(int index, int size);
        static double solvePythagoreanEquation(const double& a, const double& b);
        static double getPointToSegmentDistanceSquared(const Position2D& point,
                                                       const Position2D& firstVertex,
                                                       const Position2D& secondVertex,
                                                       Position2D& closestPoint);
        static double getPointToSegmentDistance(const Position2D& point,
                                                const Position2D& firstVertex,
                                                const Position2D& secondVertex,
                                                Position2D& closestPoint);
    private:
        static constexpr double epsilon { std::numeric_limits<double>::epsilon() };
};
