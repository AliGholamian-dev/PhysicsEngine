#include "KinematicBody.h"
#include "Exceptions.h"
#include "BodyPointerHolderIF.h"
#include "Body.h"
#include "KinematicEquationIF.h"
#include <cmath>

KinematicBody::KinematicBody(ID id,
                             Body* bodyToManipulate,
                             const sp<KinematicEquationIF>& kinematicEquation,
                             const KinematicInfo& initialKinematicInfo,
                             const DampingInfo& dampingInfo,
                             const wp<BodyPointerHolderIF>& bodyPointerHolder) :
        id(id),
        bodyToManipulate(bodyToManipulate),
        kinematicEquation(kinematicEquation),
        lastKinematicInfo(initialKinematicInfo),
        dampingInfo(dampingInfo),
        bodyPointerHolder(bodyPointerHolder)
{
    checkInputParameters();
    registerBody();
}

KinematicBody::~KinematicBody() {
    removeBody();
}

void KinematicBody::checkInputParameters() const {
    if(dampingInfo.angularDampingFactor > 1 ||
       dampingInfo.angularDampingFactor < 0 ||
       dampingInfo.linearDampingFactor > 1 ||
       dampingInfo.linearDampingFactor < 0)
    {
        throw WrongParametersException();
    }
}

const ID& KinematicBody::getKinematicBodyID() const {
    return id;
}

void KinematicBody::registerBody() {
    if(!bodyPointerHolder.expired()) {
        bodyPointerHolder.lock()->registerKinematicBody(id, this);
    }
}

void KinematicBody::removeBody() {
    if(!bodyPointerHolder.expired()) {
        bodyPointerHolder.lock()->removeKinematicBody(id);
    }
}

void KinematicBody::setDampingInfo(const DampingInfo& newDampingInfo) {
    dampingInfo = newDampingInfo;
}

void KinematicBody::setKinematicInfo(const KinematicInfo& kinematicInfo) {
    lastKinematicInfo = kinematicInfo;
}

void KinematicBody::addKinematicInfo(const KinematicInfo& kinematicInfo) {
    if(!additionalKinematicInfo) {
        additionalKinematicInfo = kinematicInfo;
    }
    else {
        additionalKinematicInfo->linearVelocity += kinematicInfo.linearVelocity;
        additionalKinematicInfo->linearAcceleration += kinematicInfo.linearAcceleration;
        additionalKinematicInfo->displacementVector += kinematicInfo.displacementVector;
        additionalKinematicInfo->angularVelocity += kinematicInfo.angularVelocity;
        additionalKinematicInfo->angularAcceleration += kinematicInfo.angularAcceleration;
        additionalKinematicInfo->rotatedAngle += kinematicInfo.rotatedAngle;
    }
}

KinematicInfo KinematicBody::getInitialState(const Vector2D& calculatedLinearAcceleration,
                                             const double& calculatedAngularAcceleration)
{
    KinematicInfo initialState { lastKinematicInfo };
    initialState.linearAcceleration = calculatedLinearAcceleration;
    initialState.displacementVector = Vector2D();
    initialState.angularAcceleration = calculatedAngularAcceleration;
    initialState.rotatedAngle = 0;
    return initialState;
}

void KinematicBody::addUserAddedKinematicInfoTo(KinematicInfo& kinematicInfo) const {
    if(additionalKinematicInfo) {
        kinematicInfo.linearVelocity += additionalKinematicInfo->linearVelocity;
        kinematicInfo.linearAcceleration += additionalKinematicInfo->linearAcceleration;
        kinematicInfo.displacementVector += additionalKinematicInfo->displacementVector;
        kinematicInfo.angularVelocity += additionalKinematicInfo->angularVelocity;
        kinematicInfo.angularAcceleration += additionalKinematicInfo->angularAcceleration;
        kinematicInfo.rotatedAngle += additionalKinematicInfo->rotatedAngle;
    }
}

void KinematicBody::moveBody(const KinematicInfo& finalState) {
    if(bodyToManipulate) {
        bodyToManipulate->changePositionBy(finalState.displacementVector);
        bodyToManipulate->rotate(finalState.rotatedAngle);
    }
}

void KinematicBody::updateLastKinematicInfo(const KinematicInfo& finalState) {
    lastKinematicInfo = finalState;
}

void KinematicBody::damp(const double& deltaTime) {
    const double linearDampingFactor = std::pow(dampingInfo.linearDampingFactor, deltaTime);
    const double angularDampingFactor = std::pow(dampingInfo.angularDampingFactor, deltaTime);
    lastKinematicInfo.linearVelocity *= linearDampingFactor;
    lastKinematicInfo.angularVelocity *= angularDampingFactor;
}

void KinematicBody::resetAdditionalKinematicInfo() {
    additionalKinematicInfo.reset();
}

void KinematicBody::manipulateBody(const Vector2D& calculatedLinearAcceleration,
                                   const double& calculatedAngularAcceleration,
                                   const double& deltaTime)
{
    KinematicInfo initialState = getInitialState(calculatedLinearAcceleration, calculatedAngularAcceleration);
    addUserAddedKinematicInfoTo(initialState);
    if(kinematicEquation) {
        auto finalState = kinematicEquation->getUpdatedKinematicInfo(initialState, deltaTime);
        moveBody(finalState);
        updateLastKinematicInfo(finalState);
        damp(deltaTime);
    }
    resetAdditionalKinematicInfo();
}

const KinematicInfo& KinematicBody::getLastKinematicInfo() const {
    return lastKinematicInfo;
}

const DampingInfo& KinematicBody::getDampingInfo() const {
    return dampingInfo;
}
