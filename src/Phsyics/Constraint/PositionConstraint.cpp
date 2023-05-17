#include "PositionConstraint.h"
#include "Vector2D.h"
#include "RigidBody.h"
#include "KinematicBody.h"
#include "Body.h"
#include <cmath>


PositionConstraint::PositionConstraint(Body* firstBody,
                                       KinematicBody* firstKinematicBody,
                                       RigidBody* firstRigidBody,
                                       Body* secondBody,
                                       KinematicBody* secondKinematicBody,
                                       RigidBody* secondRigidBody,
                                       const double& distance) :
        Constraint(),
        firstBody(firstBody),
        firstKinematicBody(firstKinematicBody),
        firstRigidBody(firstRigidBody),
        secondBody(secondBody),
        secondKinematicBody(secondKinematicBody),
        secondRigidBody(secondRigidBody),
        distance(distance) {}

void PositionConstraint::updateConstraint(const double& deltaTime) {
    Vector2D relativePos = firstBody->getCenterPosition() - secondBody->getCenterPosition();
    double currentDistance = relativePos.getMagnitude();
    double offset = distance - currentDistance ;
    if (std::abs(offset) > 0.0) {
        Vector2D offsetDir = relativePos.getNormalizedVector();
        Vector2D relativeVelocity = firstKinematicBody->getLastKinematicInfo().linearVelocity -
                secondKinematicBody->getLastKinematicInfo().linearVelocity ;
        double constraintMass = firstRigidBody->getInverseRigidInfo().inverseMass +
                secondRigidBody->getInverseRigidInfo().inverseMass;
        if (constraintMass > 0.0) {
            double velocityDot = relativeVelocity.getDotProductTo(offsetDir);
            double biasFactor = 0.01;
            double bias = -(biasFactor / deltaTime) * offset;
            double lambda = -(velocityDot + bias) / constraintMass;
            Vector2D aImpulse = offsetDir * lambda;
            Vector2D bImpulse = -1 * offsetDir * lambda;

            auto firstKinematicInfo = firstKinematicBody->getLastKinematicInfo();
            firstKinematicInfo.linearVelocity += firstRigidBody->getInverseRigidInfo().inverseMass * aImpulse;
            firstKinematicBody->setKinematicInfo(firstKinematicInfo);

            auto secondKinematicInfo = secondKinematicBody->getLastKinematicInfo();
            secondKinematicInfo.linearVelocity += secondRigidBody->getInverseRigidInfo().inverseMass * bImpulse;
            secondKinematicBody->setKinematicInfo(secondKinematicInfo);

        }
    }
}
