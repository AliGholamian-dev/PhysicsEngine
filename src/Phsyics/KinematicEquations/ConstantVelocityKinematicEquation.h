#pragma once

#include "KinematicEquationIF.h"

class ConstantVelocityKinematicEquation final : public KinematicEquationIF{
    public:
        ConstantVelocityKinematicEquation();
        ~ConstantVelocityKinematicEquation() override = default;

        [[nodiscard]] KinematicInfo getUpdatedKinematicInfo(const KinematicInfo& initialState,
                                                            const double& deltaTime) const override;

    private:
        [[nodiscard]] static Vector2D calculateDisplacement(const Vector2D& linearVelocity,
                                                            const double& deltaTime) ;
        [[nodiscard]] static double calculateRotationAngle(const double& angularVelocity,
                                                           const double& deltaTime) ;
};
