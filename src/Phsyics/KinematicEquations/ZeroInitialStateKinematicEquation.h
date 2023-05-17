#pragma once


#include "KinematicEquationIF.h"

class ZeroInitialStateKinematicEquation final : public KinematicEquationIF {
    public:
        ZeroInitialStateKinematicEquation();
        ~ZeroInitialStateKinematicEquation() override = default;

        [[nodiscard]] KinematicInfo getUpdatedKinematicInfo(const KinematicInfo& initialState,
                                                            const double& deltaTime) const override;

    private:
        [[nodiscard]] static Vector2D calculateLinearVelocity(const Vector2D& linearAcceleration,
                                                              const double& deltaTime) ;

        [[nodiscard]] static Vector2D calculateDisplacement(const Vector2D& finalLinearVelocity,
                                                            const double& deltaTime) ;

        [[nodiscard]] static double calculateAngularVelocity(const double& angularAcceleration,
                                                             const double& deltaTime) ;

        [[nodiscard]] static double calculateRotationAngle(const double& finalAngularVelocity,
                                                           const double& deltaTime) ;
};
