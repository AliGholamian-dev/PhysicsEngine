#pragma once

#include "Vector2D.h"

class AccelerationCalculatorIF {
    public:
        AccelerationCalculatorIF() = default;
        virtual ~AccelerationCalculatorIF() = default;

        [[nodiscard]] virtual Vector2D getLinearAcceleration(const Vector2D& appliedForce,
                                                             const double& deltaTime) const = 0;
        [[nodiscard]] virtual double getAngularAcceleration(const double& appliedTorque,
                                                            const double& deltaTime) const = 0;
};