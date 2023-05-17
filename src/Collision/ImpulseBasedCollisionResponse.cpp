#include "ImpulseBasedCollisionResponse.h"
#include "CollidingBody.h"
#include "RigidBody.h"
#include "KinematicBody.h"
#include "Body.h"


ImpulseBasedCollisionResponse::ImpulseBasedCollisionResponse() : CollisionResponseIF() {}

Vector2D ImpulseBasedCollisionResponse::determineLinearVelocity(const KinematicBody*kinematicBody) {
    Vector2D linearVelocity {0, 0};
    if(kinematicBody) {
        linearVelocity = kinematicBody->getLastKinematicInfo().linearVelocity;
    }
    return linearVelocity;
}

double ImpulseBasedCollisionResponse::determineAngularVelocity(const KinematicBody*kinematicBody) {
    double angularVelocity { 0 };
    if(kinematicBody) {
        angularVelocity = kinematicBody->getLastKinematicInfo().angularVelocity;
    }
    return angularVelocity;
}

Vector2D ImpulseBasedCollisionResponse::getPerpendicularAxisCrossProduct(const double& z, const Vector2D& vector) {
    return {-z * vector.getY(), z * vector.getX()};
}

Vector2D ImpulseBasedCollisionResponse::calculateContactVelocity(const CollisionResponseInfo& collisionResponseInfo,
                                                                 const Vector2D& firstBodyTorqueArm,
                                                                 const Vector2D& secondBodyTorqueArm)
{
    const auto& firstCollidingBody = collisionResponseInfo.collidingPair.first;
    const auto& secondCollidingBody = collisionResponseInfo.collidingPair.second;
    const auto& firstKinematicBody = firstCollidingBody->getKinematicBody();
    const auto& secondKinematicBody = secondCollidingBody->getKinematicBody();

    Vector2D firstBodyLinearVelocity = determineLinearVelocity(firstKinematicBody);
    Vector2D secondBodyLinearVelocity = determineLinearVelocity(secondKinematicBody);
    double firstBodyAngularVelocity = determineAngularVelocity(firstKinematicBody);
    double secondBodyAngularVelocity = determineAngularVelocity(secondKinematicBody);
    const Vector2D linearVelocityOfAngularVelocityA { getPerpendicularAxisCrossProduct(firstBodyAngularVelocity,
                                                                                       firstBodyTorqueArm) };
    const Vector2D linearVelocityOfAngularVelocityB { getPerpendicularAxisCrossProduct(secondBodyAngularVelocity,
                                                                                       secondBodyTorqueArm) };
    Vector2D fullVelocityA = firstBodyLinearVelocity + linearVelocityOfAngularVelocityA;
    Vector2D fullVelocityB = secondBodyLinearVelocity + linearVelocityOfAngularVelocityB;
    Vector2D contactVelocity = fullVelocityB - fullVelocityA;
    return contactVelocity;
}

Vector2D ImpulseBasedCollisionResponse::calculateImpulse(const CollisionResponseInfo& collisionResponseInfo,
                                                         const Vector2D& impulseDirectionUnitVector,
                                                         const Vector2D& contactVelocity,
                                                         const Vector2D& firstBodyTorqueArm,
                                                         const Vector2D& secondBodyTorqueArm)
{
    const auto& firstCollidingBody = collisionResponseInfo.collidingPair.first;
    const auto& secondCollidingBody = collisionResponseInfo.collidingPair.second;
    const auto& firstRigidBody = firstCollidingBody->getRigidBody();
    const auto& secondRigidBody = secondCollidingBody->getRigidBody();
    const auto inverseRotationalInertiaA = firstRigidBody->getInverseRigidInfo().inverseRotationalInertia;
    const auto inverseRotationalInertiaB = secondRigidBody->getInverseRigidInfo().inverseRotationalInertia;
    const auto& firstBodyInverseMass = firstRigidBody->getInverseRigidInfo().inverseMass;
    const auto& secondBodyInverseMass = secondRigidBody->getInverseRigidInfo().inverseMass;

    const double velocityProjectionMagnitude = contactVelocity.getDotProductTo(impulseDirectionUnitVector);
    const double torqueArmAProjection = firstBodyTorqueArm.getCrossProductTo(impulseDirectionUnitVector);
    const double torqueArmBProjection = secondBodyTorqueArm.getCrossProductTo(impulseDirectionUnitVector);
    const double modifiedInverseRotationalInertiaA = (torqueArmAProjection * torqueArmAProjection) * inverseRotationalInertiaA;
    const double modifiedInverseRotationalInertiaB = (torqueArmBProjection * torqueArmBProjection) * inverseRotationalInertiaB;
    const double sumInverseRotationalInertia = modifiedInverseRotationalInertiaA + modifiedInverseRotationalInertiaB;
    const double sumInverseMass = firstBodyInverseMass + secondBodyInverseMass;
    const double collisionRestitution = std::min(firstRigidBody->getRigidInfo().restitution,
                                                 secondRigidBody->getRigidInfo().restitution);

    double j = (-(1.0 + collisionRestitution) * velocityProjectionMagnitude) / (sumInverseMass + sumInverseRotationalInertia);
    return impulseDirectionUnitVector * j;
}

double ImpulseBasedCollisionResponse::calculateAngularImpulse(const Vector2D& torqueArm, const Vector2D& impulse) {
    return torqueArm.getCrossProductTo(impulse);
}

Vector2D ImpulseBasedCollisionResponse::clampFrictionImpulse(const double& frictionImpulseForce,
                                                             const double& collisionImpulseForce,
                                                             const Vector2D& direction,
                                                             const RigidInfo& firstBodyRigidInfo,
                                                             const RigidInfo& secondBodyRigidInfo)
{
    Vector2D frictionImpulse;

    double mu = std::min(firstBodyRigidInfo.staticFriction,
                         secondBodyRigidInfo.staticFriction);
    if(frictionImpulseForce < collisionImpulseForce * mu) {
        frictionImpulse = frictionImpulseForce * direction;
    }
    else {
        double dynamicFriction = std::min(firstBodyRigidInfo.dynamicFriction,
                                          secondBodyRigidInfo.dynamicFriction);
        frictionImpulse = collisionImpulseForce * dynamicFriction * direction;
    }

    return -1 * frictionImpulse;
}

void ImpulseBasedCollisionResponse::applyLinearAndAngularImpulse(KinematicBody* kinematicBody,
                                                                 const Vector2D& linearImpulse,
                                                                 const double& angularImpulse,
                                                                 const double& inverseMass,
                                                                 const double& inverseRotationalInertia) {
    if(kinematicBody) {
        auto kinematicInfo = kinematicBody->getLastKinematicInfo();
        kinematicInfo.linearVelocity += inverseMass * linearImpulse;
        kinematicInfo.angularVelocity += inverseRotationalInertia * angularImpulse;
        kinematicBody->setKinematicInfo(kinematicInfo);
    }
}

void ImpulseBasedCollisionResponse::respondToEachCollidingPair(const CollisionResponseInfo& collisionResponseInfo,
                                                               const double& deltaTime)
{
    const auto& firstCollidingBody = collisionResponseInfo.collidingPair.first;
    const auto& secondCollidingBody = collisionResponseInfo.collidingPair.second;
    const auto& firstBody = firstCollidingBody->getBody();
    const auto& secondBody = secondCollidingBody->getBody();
    const auto& firstRigidBody = firstCollidingBody->getRigidBody();
    const auto& secondRigidBody = secondCollidingBody->getRigidBody();
    const auto& firstKinematicBody = firstCollidingBody->getKinematicBody();
    const auto& secondKinematicBody = secondCollidingBody->getKinematicBody();
    const auto inverseRotationalInertiaA = firstRigidBody->getInverseRigidInfo().inverseRotationalInertia;
    const auto inverseRotationalInertiaB = secondRigidBody->getInverseRigidInfo().inverseRotationalInertia;
    const auto& firstBodyInverseMass = firstRigidBody->getInverseRigidInfo().inverseMass;
    const auto& secondBodyInverseMass = secondRigidBody->getInverseRigidInfo().inverseMass;

    Vector2D firstBodyLinearImpulse;
    Vector2D secondBodyLinearImpulse;
    double firstBodyAngularImpulse = 0;
    double secondBodyAngularImpulse = 0;

    for(const auto& penetrationContactPointsPair : collisionResponseInfo.penetrationContactPointsPairs) {
        const Vector2D penetrationNormal = penetrationContactPointsPair.penetrationVector.getNormalizedVector();
        for(const auto& contactPoint : penetrationContactPointsPair.contactPoints) {
            const Vector2D firstBodyTorqueArm  = contactPoint - firstBody->getCenterPosition();
            const Vector2D secondBodyTorqueArm = contactPoint - secondBody->getCenterPosition();
            Vector2D contactVelocity = calculateContactVelocity(collisionResponseInfo, firstBodyTorqueArm, secondBodyTorqueArm);
            Vector2D impulse = calculateImpulse(collisionResponseInfo,
                                                penetrationNormal,
                                                contactVelocity,
                                                firstBodyTorqueArm,
                                                secondBodyTorqueArm);

            const auto reverseImpulse { -1 * impulse };
            const double firstBodyCurrentAngularImpulse = calculateAngularImpulse(firstBodyTorqueArm, reverseImpulse);
            const double secondBodyCurrentAngularImpulse = calculateAngularImpulse(secondBodyTorqueArm, impulse);


            firstBodyLinearImpulse += reverseImpulse;
            secondBodyLinearImpulse += impulse;
            firstBodyAngularImpulse += firstBodyCurrentAngularImpulse;
            secondBodyAngularImpulse += secondBodyCurrentAngularImpulse;


            KinematicInfo firstKinematicInfoCopy;
            KinematicInfo secondKinematicInfoCopy;
            if(firstKinematicBody) {
                firstKinematicInfoCopy = firstKinematicBody->getLastKinematicInfo();
            }
            if(secondKinematicBody) {
                secondKinematicInfoCopy = secondKinematicBody->getLastKinematicInfo();
            }
            applyLinearAndAngularImpulse(firstKinematicBody,
                                         reverseImpulse,
                                         firstBodyCurrentAngularImpulse,
                                         firstBodyInverseMass,
                                         inverseRotationalInertiaA);
            applyLinearAndAngularImpulse(secondKinematicBody,
                                         impulse,
                                         secondBodyCurrentAngularImpulse,
                                         secondBodyInverseMass,
                                         inverseRotationalInertiaB);

            contactVelocity = calculateContactVelocity(collisionResponseInfo, firstBodyTorqueArm, secondBodyTorqueArm);
            if(firstKinematicBody) {
                firstKinematicBody->setKinematicInfo(firstKinematicInfoCopy);
            }
            if(secondKinematicBody) {
                secondKinematicBody->setKinematicInfo(secondKinematicInfoCopy);
            }

            Vector2D impulseDirectionUnitVector =
                contactVelocity - (contactVelocity.getDotProductTo(penetrationNormal) * penetrationNormal);
            impulseDirectionUnitVector = impulseDirectionUnitVector.getNormalizedVector();
            Vector2D frictionImpulse = calculateImpulse(collisionResponseInfo,
                                                        impulseDirectionUnitVector,
                                                        contactVelocity,
                                                        firstBodyTorqueArm,
                                                        secondBodyTorqueArm);
            frictionImpulse = clampFrictionImpulse(frictionImpulse.getMagnitude(),
                                                   impulse.getMagnitude(),
                                                   impulseDirectionUnitVector,
                                                   firstRigidBody->getRigidInfo(),
                                                   secondRigidBody->getRigidInfo());
            firstBodyLinearImpulse += -1 * frictionImpulse;
            secondBodyLinearImpulse += frictionImpulse;
            firstBodyAngularImpulse += calculateAngularImpulse(firstBodyTorqueArm, -1 * frictionImpulse);
            secondBodyAngularImpulse += calculateAngularImpulse(secondBodyTorqueArm, frictionImpulse);

        }

    }

    applyLinearAndAngularImpulse(firstKinematicBody,
                                 firstBodyLinearImpulse,
                                 firstBodyAngularImpulse,
                                 firstBodyInverseMass,
                                 inverseRotationalInertiaA);
    applyLinearAndAngularImpulse(secondKinematicBody,
                                 secondBodyLinearImpulse,
                                 secondBodyAngularImpulse,
                                 secondBodyInverseMass,
                                 inverseRotationalInertiaB);
}
