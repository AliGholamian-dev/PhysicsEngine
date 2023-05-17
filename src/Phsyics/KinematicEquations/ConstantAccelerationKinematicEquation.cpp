#include "ConstantAccelerationKinematicEquation.h"

ConstantAccelerationKinematicEquation::ConstantAccelerationKinematicEquation() : KinematicEquationIF() {}

Vector2D ConstantAccelerationKinematicEquation::calculateLinearVelocity(const Vector2D& initialLinearVelocity,
                                                                        const Vector2D& linearAcceleration,
                                                                        const double& deltaTime)
{
    return initialLinearVelocity + (linearAcceleration * deltaTime);
}

Vector2D ConstantAccelerationKinematicEquation::calculateDisplacement(const Vector2D& initialLinearVelocity,
                                                                      const Vector2D& finalLinearVelocity,
                                                                      const double& deltaTime)
{
    return ((initialLinearVelocity + finalLinearVelocity) / 2) * deltaTime;
}

double ConstantAccelerationKinematicEquation::calculateAngularVelocity(const double& initialAngularVelocity,
                                                                       const double& angularAcceleration,
                                                                       const double& deltaTime)
{
    return initialAngularVelocity + (angularAcceleration * deltaTime);
}

double ConstantAccelerationKinematicEquation::calculateRotationAngle(const double& initialAngularVelocity,
                                                                     const double& finalAngularVelocity,
                                                                     const double& deltaTime)
{
    return ((initialAngularVelocity + finalAngularVelocity) / 2) * deltaTime;
}

KinematicInfo ConstantAccelerationKinematicEquation::getUpdatedKinematicInfo(const KinematicInfo& initialState,
                                                                             const double& deltaTime) const
{
    KinematicInfo finalState(initialState);
    finalState.linearVelocity = calculateLinearVelocity(initialState.linearVelocity,
                                                        initialState.linearAcceleration,
                                                        deltaTime);

    finalState.displacementVector = calculateDisplacement(initialState.linearVelocity,
                                                          finalState.linearVelocity,
                                                          deltaTime);

    finalState.angularVelocity = calculateAngularVelocity(initialState.angularVelocity,
                                                          initialState.angularAcceleration,
                                                          deltaTime);

    finalState.rotatedAngle = calculateRotationAngle(initialState.angularVelocity,
                                                     finalState.angularAcceleration,
                                                     deltaTime);
    return finalState;
}
