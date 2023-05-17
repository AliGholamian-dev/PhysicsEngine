#include "ZeroInitialStateKinematicEquation.h"

ZeroInitialStateKinematicEquation::ZeroInitialStateKinematicEquation() : KinematicEquationIF() {}

Vector2D ZeroInitialStateKinematicEquation::calculateLinearVelocity(const Vector2D& linearAcceleration,
                                                                    const double& deltaTime)
{
    return linearAcceleration * deltaTime;
}

Vector2D ZeroInitialStateKinematicEquation::calculateDisplacement(const Vector2D& finalLinearVelocity,
                                                                  const double& deltaTime)
{
    return finalLinearVelocity * deltaTime;
}

double ZeroInitialStateKinematicEquation::calculateAngularVelocity(const double& angularAcceleration,
                                                                   const double& deltaTime)
{
    return angularAcceleration * deltaTime;
}

double ZeroInitialStateKinematicEquation::calculateRotationAngle(const double& finalAngularVelocity,
                                                                 const double& deltaTime)
{
    return finalAngularVelocity * deltaTime;
}

KinematicInfo ZeroInitialStateKinematicEquation::getUpdatedKinematicInfo(const KinematicInfo& initialState,
                                                                         const double& deltaTime) const
{
    KinematicInfo finalState(initialState);
    finalState.linearVelocity += calculateLinearVelocity(initialState.linearAcceleration,
                                                         deltaTime);

    finalState.displacementVector = calculateDisplacement(finalState.linearVelocity,
                                                          deltaTime);

    finalState.angularVelocity += calculateAngularVelocity(initialState.angularAcceleration,
                                                           deltaTime);

    finalState.rotatedAngle = calculateRotationAngle(finalState.angularVelocity,
                                                     deltaTime);
    return finalState;
}
