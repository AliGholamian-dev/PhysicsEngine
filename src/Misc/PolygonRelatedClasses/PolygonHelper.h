#pragma once


#include "Position2D.h"
#include "TriangleInfo.h"
#include <vector>

class PolygonHelper final {
    public:
        PolygonHelper() = default;
        ~PolygonHelper()  = default;

        static std::vector<TriangleInfo> getTriangles(const std::vector<Position2D>& vertices);
        static double getArea(const std::vector<TriangleInfo>& triangles);
        static double getArea(const std::vector<Position2D>& vertices);
        static double getArea(const double& mass, const double& density);
        static double getMass(const double& area, const double& density);
        static double getMass(const std::vector<TriangleInfo>& triangles, const double& density);
        static double getMass(const std::vector<Position2D>& vertices, const double& density);
        static double getDensity(const double& area, const double& mass);
        static double getDensity(const std::vector<TriangleInfo>& triangles, const double& mass);
        static double getDensity(const std::vector<Position2D>& vertices, const double& mass);
        static Position2D getCentroid(const std::vector<Position2D>& vertices, const std::vector<TriangleInfo>& triangles);
        static Position2D getCentroid(const std::vector<Position2D>& vertices);


        static double getMomentOfInertiaByRelativeVertices(const std::vector<Position2D>& relativeVerticesToCentroid,
                                                           const double& density);
        static double getMomentOfInertiaByAbsoluteVertices(const std::vector<Position2D>& absoluteVertices,
                                                           const double& density);
        static double getMomentOfInertiaByAbsoluteVertices(const Position2D& centroid,
                                                           const std::vector<Position2D>& absoluteVertices,
                                                           const double& density);

        static std::vector<Position2D> getRelativeVertices(const Position2D& centroid,
                                                           const std::vector<Position2D>& absoluteVertices);
        static std::vector<Position2D> getRelativeVertices(const std::vector<Position2D>& absoluteVertices);
        static std::vector<Position2D> getAbsoluteVertices(const Position2D& centroid,
                                                           const std::vector<Position2D>& relativeVertices);

    private:
        [[nodiscard]] static bool checkNumberOfVertices(const std::vector<Position2D>& vertices) ;
        [[nodiscard]] static bool checkForUniqueVertices(const std::vector<Position2D>& vertices) ;
        [[nodiscard]] static bool checkForNonCollinearEdges(const std::vector<Position2D>& vertices) ;
        static void checkPolygonSpecs(const std::vector<Position2D>& vertices) ;
};
