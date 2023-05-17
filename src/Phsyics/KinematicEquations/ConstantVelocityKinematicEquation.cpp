#include "ConstantVelocityKinematicEquation.h"

ConstantVelocityKinematicEquation::ConstantVelocityKinematicEquation() : KinematicEquationIF() {}



Vector2D ConstantVelocityKinematicEquation::calculateDisplacement(const Vector2D& linearVelocity,
                                                                  const double& deltaTime)
{
    return linearVelocity * deltaTime;
}

double ConstantVelocityKinematicEquation::calculateRotationAngle(const double& angularVelocity,
                                                                 const double& deltaTime)
{
    return angularVelocity * deltaTime;
}

KinematicInfo ConstantVelocityKinematicEquation::getUpdatedKinematicInfo(const KinematicInfo& initialState,
                                                                         const double& deltaTime) const
{
    KinematicInfo finalState(initialState);

    finalState.displacementVector = calculateDisplacement(initialState.linearVelocity,
                                                          deltaTime);

    finalState.rotatedAngle = calculateRotationAngle(initialState.angularVelocity,
                                                     deltaTime);
    return finalState;
}
