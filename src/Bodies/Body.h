#pragma once

#include "PointerHelper.h"
#include "ID.h"
#include "Position2D.h"
#include "TriangleInfo.h"
#include <vector>
class BodyPointerHolderIF;

class Body {
    public:
        enum class BodyType
        {
            polygon = 0,
            circle = 1
        };

        enum class GeometryType {
            convex = 0,
            concave = 1,
        };

        Body() = delete;
        Body(const Body& other) = delete;
        Body(Body&& other) noexcept  = delete;
        Body(ID id,
             const BodyType& bodyType,
             const Position2D& initialCenterPosition,
             const double& currentRotationOfVertices,
             const std::vector<Position2D>& clockwiseVerticesRelativeToCenter,
             const double& radius,
             const std::vector<TriangleInfo>& triangles,
             const wp<BodyPointerHolderIF>& bodyPointerHolder);
        virtual ~Body();

        void setCenterPosition(const Position2D& newPosition);
        void setRotation(const double& newRotation);
        void changePositionBy(const Vector2D& displacementVector);
        void rotate(const double& rotationAngle);
        void rotateAroundCustomPoint(const Position2D& point, const double& rotationAngle);

        [[nodiscard]] const ID& getBodyID() const;
        [[nodiscard]] const BodyType& getBodyType() const;
        [[nodiscard]] const GeometryType& getGeometryType() const;
        [[nodiscard]] const Position2D& getCenterPosition() const;
        [[nodiscard]] const double& getRotation() const;
        [[nodiscard]] const std::vector<Position2D>& getRelativeVertices() const;
        [[nodiscard]] const std::vector<Position2D>& getAbsoluteVertices() const;
        [[nodiscard]] const double& getRadius() const;
        [[nodiscard]] const std::vector<TriangleInfo>& getTriangles() const;

    private:
        void registerBody();
        void removeBody();
        [[nodiscard]] bool checkBodyType() const;
        [[nodiscard]] bool checkNumberOfVertices() const;
        [[nodiscard]] bool checkForUniqueVertices() const;
        [[nodiscard]] bool checkRadius() const;
        [[nodiscard]] bool checkForNonCollinearEdges() const;
        [[nodiscard]] bool checkNumberOfTriangles() const;
        static GeometryType determineGeometryType(const std::vector<Position2D>& vertices);
        void checkInputParameters() const;
        void adjustInputParameters();
        void adjustRotationToUpperBound();
        void adjustRotationToLowerBound();
        void correctRotation();
        void requestAbsoluteVerticesUpdate();

        const ID id;
        const BodyType bodyType;
        const GeometryType geometryType;
        Position2D centerPosition;
        double rotation;
        std::vector<Position2D> clockwiseVerticesRelativeToCenter;
        mutable std::vector<Position2D> clockwiseAbsoluteVertices;
        double radius;
        const std::vector<TriangleInfo> triangles;
        wp<BodyPointerHolderIF> bodyPointerHolder;

        mutable bool absoluteVerticesNeedUpdate { true };
};
