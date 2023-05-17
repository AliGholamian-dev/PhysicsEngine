#include "RealisticEngine.h"
#include "RigidBody.h"

RealisticEngine::RealisticEngine(RunMode runMode,
                                 int numberOfIterations,
                                 int numberOfConstraintIterations,
                                 const Vector2D& gravity,
                                 const sp<CollisionDetectorIF>& collisionDetectorIF,
                                 const sp<CollisionResolverIF>& collisionResolverIF,
                                 const sp<CollisionResponseIF>& collisionResponseIF,
                                 const wp<MapBasedBodyPointerHolder>& mapBasedBodyPointerHolder) :
        EngineIF(runMode,
                 numberOfIterations,
                 numberOfConstraintIterations,
                 collisionDetectorIF,
                 collisionResolverIF,
                 collisionResponseIF,
                 mapBasedBodyPointerHolder),
        gravity(gravity),
        mapBasedBodyPointerHolder(mapBasedBodyPointerHolder) {}

void RealisticEngine::applyGravity(RigidBody* rigidBody) {
    if(rigidBody->getRigidBodyType() == RigidBody::RigidBodyType::notStaticBody) {
        const double gravityScale = rigidBody->getGravityScale();
        const double bodyMass = rigidBody->getRigidInfo().mass;
        const auto gravityForce = bodyMass * gravityScale * gravity;
        rigidBody->addForce(gravityForce);
    }
}

void RealisticEngine::manipulateRigidBody(RigidBody* rigidBody, const double& deltaTime) {
    rigidBody->manipulateKinematicBody(deltaTime);
}

void RealisticEngine::prepareUserControlledBodies()
{
    if(!mapBasedBodyPointerHolder.expired()) {
        bodyHolderForUserControlledBodies = mapBasedBodyPointerHolder.lock();
        if(bodyHolderForUserControlledBodies) {
            userControlledIt = bodyHolderForUserControlledBodies->userControlledBodiesMap.begin();
        }
    }
}

bool RealisticEngine::checkForNextUserControlledBodyAvailable()
{
    bool notReachedEnd { false };
    if(bodyHolderForUserControlledBodies) {
        notReachedEnd = userControlledIt != bodyHolderForUserControlledBodies->userControlledBodiesMap.end();
    }
    if(!notReachedEnd) {
        bodyHolderForUserControlledBodies.reset();
    }
    return notReachedEnd;
}

UserControlledBody* RealisticEngine::provideNextUserControlledBod()
{
    if(bodyHolderForUserControlledBodies) {
        auto userControlledBody = userControlledIt->second;
        userControlledIt++;
        return userControlledBody;
    }
    return nullptr;
}

void RealisticEngine::prepareBodiesForUpdate() {
    if(!mapBasedBodyPointerHolder.expired()) {
        bodyHolderForRigidBodies = mapBasedBodyPointerHolder.lock();
        if(bodyHolderForRigidBodies) {
            rigidIt = bodyHolderForRigidBodies->rigidBodiesMap.begin();
        }
    }
}

bool RealisticEngine::checkForNextBodyAvailableForUpdate() {
    bool notReachedEnd { false };
    if(bodyHolderForRigidBodies) {
        notReachedEnd = rigidIt != bodyHolderForRigidBodies->rigidBodiesMap.end();
    }
    if(!notReachedEnd) {
        bodyHolderForRigidBodies.reset();
    }
    return notReachedEnd;
}

void RealisticEngine::updateNextBody(const double& deltaTime) {
    if(bodyHolderForRigidBodies) {
        const auto& rigidBody = rigidIt->second;
        applyGravity(rigidBody);
        manipulateRigidBody(rigidBody, deltaTime);
        rigidIt++;
    }
}
