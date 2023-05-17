#pragma once

#include "KinematicEquationIF.h"

class ConstantAccelerationKinematicEquation final : public KinematicEquationIF {
    public:
        ConstantAccelerationKinematicEquation();
        ~ConstantAccelerationKinematicEquation() override = default;

        [[nodiscard]] KinematicInfo getUpdatedKinematicInfo(const KinematicInfo& initialState,
                                                            const double& deltaTime) const override;

    private:
        [[nodiscard]] static Vector2D calculateLinearVelocity(const Vector2D& initialLinearVelocity,
                                                              const Vector2D& linearAcceleration,
                                                              const double& deltaTime) ;

        [[nodiscard]] static Vector2D calculateDisplacement(const Vector2D& initialLinearVelocity,
                                                            const Vector2D& finalLinearVelocity,
                                                            const double& deltaTime) ;

        [[nodiscard]] static double calculateAngularVelocity(const double& initialAngularVelocity,
                                                             const double& angularAcceleration,
                                                             const double& deltaTime) ;

        [[nodiscard]] static double calculateRotationAngle(const double& initialAngularVelocity,
                                                           const double& finalAngularVelocity,
                                                           const double& deltaTime) ;
};
