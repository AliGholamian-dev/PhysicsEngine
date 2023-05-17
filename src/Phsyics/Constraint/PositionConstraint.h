#pragma once


#include "Constraint.h"
class Body;
class KinematicBody;
class RigidBody;

class PositionConstraint final : public Constraint {
    public:
        PositionConstraint(Body* firstBody,
                           KinematicBody* firstKinematicBody,
                           RigidBody* firstRigidBody,
                           Body* secondBody,
                           KinematicBody* secondKinematicBody,
                           RigidBody* secondRigidBody,
                           const double& distance);
        ~PositionConstraint() override = default;

        void updateConstraint(const double& deltaTime) override;

    private:
        Body* firstBody;
        KinematicBody* firstKinematicBody;
        RigidBody* firstRigidBody;
        Body* secondBody;
        KinematicBody* secondKinematicBody;
        RigidBody* secondRigidBody;
        double distance;
};
