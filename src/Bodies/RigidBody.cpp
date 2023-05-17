#include "RigidBody.h"
#include "BodyPointerHolderIF.h"
#include "KinematicBody.h"
#include "AccelerationCalculatorIF.h"
#include "Exceptions.h"
#include "MathUtils.h"

RigidBody::RigidBody(ID id,
                     const RigidBodyType& rigidBodyType,
                     KinematicBody* kinematicBodyToManipulate,
                     const sp<AccelerationCalculatorIF>& accelerationCalculator,
                     const RigidInfo& rigidInfo,
                     const double& gravityScale,
                     const wp<BodyPointerHolderIF>& bodyPointerHolder) :
        id(id),
        rigidBodyType(rigidBodyType),
        kinematicBodyToManipulate(kinematicBodyToManipulate),
        accelerationCalculator(accelerationCalculator),
        rigidInfo(rigidInfo),
        inverseRigidInfo({0, 0, 0 ,0, 0, 0, 0}),
        gravityScale(gravityScale),
        bodyPointerHolder(bodyPointerHolder)
{
    checkInputParameters();
    fillInverseRigidInfo();
    registerBody();
}

RigidBody::~RigidBody() {
    removeBody();
}

const ID& RigidBody::getRigidBodyID() const {
    return id;
}

void RigidBody::registerBody() {
    if(!bodyPointerHolder.expired()) {
        bodyPointerHolder.lock()->registerRigidBody(id, this);
    }
}

void RigidBody::removeBody() {
    if(!bodyPointerHolder.expired()) {
        bodyPointerHolder.lock()->removeRigidBody(id);
    }
}

bool RigidBody::checkForInRangeInputs() const {
    if(rigidInfo.mass < 0 ||
       rigidInfo.area < 0 ||
       rigidInfo.density < 0 ||
       rigidInfo.restitution < 0 ||
       rigidInfo.restitution > 1 ||
       rigidInfo.rotationalInertia < 0 ||
       rigidInfo.staticFriction > 1 ||
       rigidInfo.staticFriction < 0 ||
       rigidInfo.dynamicFriction > 1 ||
       rigidInfo.dynamicFriction < 0 ||
       MathUtils::checkForEquality(rigidInfo.mass, 0) ||
       MathUtils::checkForEquality(rigidInfo.area, 0) ||
       MathUtils::checkForEquality(rigidInfo.density, 0) ||
       MathUtils::checkForEquality(rigidInfo.restitution,0) ||
       MathUtils::checkForEquality(rigidInfo.rotationalInertia,0))
    {
        return false;
    }
    return true;
}

bool RigidBody::checkForCorrespondingInputs() const {
    if(!MathUtils::checkForEquality((rigidInfo.area * rigidInfo.density), rigidInfo.mass, 0.1)) {
        return false;
    }
    if(rigidInfo.dynamicFriction > rigidInfo.staticFriction) {
        return false;
    }
    return true;
}

void RigidBody::checkInputParameters() const {
    bool inputParametersAreOk = checkForInRangeInputs();
    inputParametersAreOk = inputParametersAreOk && checkForCorrespondingInputs();
    if(!inputParametersAreOk) {
        throw WrongParametersException();
    }
}

void RigidBody::fillInverseRigidInfo() {
    if(rigidBodyType == RigidBodyType::notStaticBody) {
        inverseRigidInfo.inverseMass = 1 / rigidInfo.mass;
        inverseRigidInfo.inverseArea = 1 / rigidInfo.area;
        inverseRigidInfo.inverseDensity = 1 / rigidInfo.density;
        inverseRigidInfo.inverseRestitution = 1 / rigidInfo.restitution;
        inverseRigidInfo.inverseRotationalInertia = 1 / rigidInfo.rotationalInertia;
    }
    inverseRigidInfo.inverseStaticFriction = 1 / rigidInfo.staticFriction;
    inverseRigidInfo.inverseDynamicFriction = 1 / rigidInfo.dynamicFriction;
}

void RigidBody::addForce(const Vector2D& force) {
    sumOfAppliedForces += force;
}

void RigidBody::addTorque(const double& torque) {
    sumOfAppliedTorques += torque;
}

void RigidBody::resetSumOfAppliedForces() {
    sumOfAppliedForces = Vector2D();
}

void RigidBody::resetSumOfAppliedTorques() {
    sumOfAppliedTorques = 0;
}

void RigidBody::manipulateKinematicBody(const double& deltaTime) {
    if(kinematicBodyToManipulate) {
        const Vector2D linearAcceleration = accelerationCalculator->getLinearAcceleration(sumOfAppliedForces, deltaTime);
        const double angularAcceleration = accelerationCalculator->getAngularAcceleration(sumOfAppliedTorques, deltaTime);
        kinematicBodyToManipulate->manipulateBody(linearAcceleration, angularAcceleration, deltaTime);
    }
    resetSumOfAppliedForces();
    resetSumOfAppliedTorques();
}

const RigidBody::RigidBodyType& RigidBody::getRigidBodyType() const {
    return rigidBodyType;
}

const RigidInfo& RigidBody::getRigidInfo() const {
    return rigidInfo;
}

const InverseRigidInfo& RigidBody::getInverseRigidInfo() const {
    return inverseRigidInfo;
}

const double& RigidBody::getGravityScale() const {
    return gravityScale;
}
