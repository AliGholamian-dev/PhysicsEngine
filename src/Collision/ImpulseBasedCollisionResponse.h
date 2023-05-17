#pragma once


#include "CollisionResponseIF.h"
#include "RigidInfo.h"
class KinematicBody;


class ImpulseBasedCollisionResponse final : public CollisionResponseIF {
    public:
        ImpulseBasedCollisionResponse();
        ~ImpulseBasedCollisionResponse() override = default;

        void respondToEachCollidingPair(const CollisionResponseInfo& collisionResponseInfo,
                                        const double& deltaTime) override;

    private:
        static Vector2D determineLinearVelocity(const KinematicBody* kinematicBody);
        static double determineAngularVelocity(const KinematicBody* kinematicBody);
        static Vector2D getPerpendicularAxisCrossProduct(const double& z, const Vector2D& vector);
        static Vector2D calculateContactVelocity(const CollisionResponseInfo& collisionResponseInfo,
                                                 const Vector2D& firstBodyTorqueArm,
                                                 const Vector2D& secondBodyTorqueArm);
        static Vector2D calculateImpulse(const CollisionResponseInfo& collisionResponseInfo,
                                         const Vector2D& impulseDirectionUnitVector,
                                         const Vector2D& contactVelocity,
                                         const Vector2D& firstBodyTorqueArm,
                                         const Vector2D& secondBodyTorqueArm);
        static double calculateAngularImpulse(const Vector2D& torqueArm, const Vector2D& impulse);
        static void applyLinearAndAngularImpulse(KinematicBody* kinematicBody,
                                                 const Vector2D& linearImpulse,
                                                 const double& angularImpulse,
                                                 const double& inverseMass,
                                                 const double& inverseRotationalInertia);
        static Vector2D clampFrictionImpulse(const double& frictionImpulseForce,
                                             const double& collisionImpulseForce,
                                             const Vector2D& direction,
                                             const RigidInfo& firstBodyRigidInfo,
                                             const RigidInfo& secondBodyRigidInfo);
};
