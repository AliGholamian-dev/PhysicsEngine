#pragma once

#include "AccelerationCalculatorIF.h"
class Body;
class RigidBody;

class NewtonAccelerationCalculator final : public AccelerationCalculatorIF {
    public:
        NewtonAccelerationCalculator() = delete;
        NewtonAccelerationCalculator(const Body* body, const RigidBody* rigidBody);
        ~NewtonAccelerationCalculator() override = default;

        [[nodiscard]] Vector2D getLinearAcceleration(const Vector2D& appliedForce,
                                                     const double& deltaTime) const override;

        [[nodiscard]] double getAngularAcceleration(const double& appliedTorque,
                                                    const double& deltaTime) const override;

    private:
        const Body* body;
        const RigidBody* rigidBody;

};
