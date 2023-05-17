#include "NewtonAccelerationCalculator.h"
#include "Body.h"
#include "RigidBody.h"


NewtonAccelerationCalculator::NewtonAccelerationCalculator(const Body* body, const RigidBody* rigidBody)
        : AccelerationCalculatorIF(),
          body(body),
          rigidBody(rigidBody) {}

Vector2D NewtonAccelerationCalculator::getLinearAcceleration(const Vector2D& appliedForce,
                                                             const double& deltaTime) const
{
    if(rigidBody) {
        return appliedForce * rigidBody->getInverseRigidInfo().inverseMass;
    }
    return {0, 0};
}

double NewtonAccelerationCalculator::getAngularAcceleration(const double& appliedTorque,
                                                            const double& deltaTime) const
{
    if(rigidBody) {
        return appliedTorque * rigidBody->getInverseRigidInfo().inverseRotationalInertia;
    }
    return 0;
}
