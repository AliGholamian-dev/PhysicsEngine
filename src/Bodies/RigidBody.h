#pragma once


#include "Vector2D.h"
#include "RigidInfo.h"
#include "PointerHelper.h"
#include "ID.h"

class KinematicBody;
class AccelerationCalculatorIF;
class BodyPointerHolderIF;

class RigidBody {
    public:
        enum class RigidBodyType {
                staticBody,
                notStaticBody,
        };
        RigidBody() = delete;
        RigidBody(const RigidBody& other) = delete;
        RigidBody(RigidBody&& other) noexcept  = delete;
        RigidBody(ID id,
                  const RigidBodyType& rigidBodyType,
                  KinematicBody* kinematicBodyToManipulate,
                  const sp<AccelerationCalculatorIF>& accelerationCalculator,
                  const RigidInfo& rigidInfo,
                  const double& gravityScale,
                  const wp<BodyPointerHolderIF>& bodyPointerHolder);
        virtual ~RigidBody();
        [[nodiscard]] const ID& getRigidBodyID() const;

        void addForce(const Vector2D& force);
        void addTorque(const double& torque);
        void manipulateKinematicBody(const double& deltaTime);
        [[nodiscard]] const RigidBodyType& getRigidBodyType() const;
        [[nodiscard]] const RigidInfo& getRigidInfo() const;
        [[nodiscard]] const InverseRigidInfo& getInverseRigidInfo() const;
        [[nodiscard]] const double& getGravityScale() const;

    private:
        [[nodiscard]] bool checkForInRangeInputs() const;
        [[nodiscard]] bool checkForCorrespondingInputs() const;
        void checkInputParameters() const;
        void fillInverseRigidInfo();
        void registerBody();
        void removeBody();
        void resetSumOfAppliedForces();
        void resetSumOfAppliedTorques();

        const ID id;
        const RigidBodyType rigidBodyType;
        KinematicBody* kinematicBodyToManipulate;
        sp<AccelerationCalculatorIF> accelerationCalculator;
        const RigidInfo rigidInfo;
        InverseRigidInfo inverseRigidInfo;
        const double gravityScale;
        wp<BodyPointerHolderIF> bodyPointerHolder;

        Vector2D sumOfAppliedForces;
        double sumOfAppliedTorques{ 0 };
};
