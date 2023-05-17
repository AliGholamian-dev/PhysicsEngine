#pragma once


#include "OneTimeUsable.h"
#include "TriangleInfo.h"
#include "Position2D.h"
#include <vector>

class PolygonTriangulation final :  private OneTimeUsable {
    public:
        explicit PolygonTriangulation(const std::vector<Position2D>& vertices);
        ~PolygonTriangulation() override = default;
        std::vector<TriangleInfo> triangulate();

    private:
        void fillIndexes();
        bool checkForConvexAngle();
        bool checkForNoVertexInTriangle();

        bool checkValidEarConditions();
        std::vector<TriangleInfo> getTrianglesByEarCutting();

        const std::vector<Position2D>& vertices;
        std::vector<int> indexes;

        int currentIndex { 0 };
        int prevIndex { 0 };
        int nextIndex { 0 };
};
