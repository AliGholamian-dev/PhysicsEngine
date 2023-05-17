#include "MapBasedBodyPointerHolder.h"
#include "UserControlledBody.h"
#include "CollidingBody.h"
#include "RigidBody.h"
#include "KinematicBody.h"
#include "Body.h"


MapBasedBodyPointerHolder::MapBasedBodyPointerHolder() : BodyPointerHolderIF() {}

void MapBasedBodyPointerHolder::queueBodyForDeletion(const ID& id) {
    toBeDeletedBodies.push_back(id);
}

void MapBasedBodyPointerHolder::queueKinematicBodyForDeletion(const ID& id) {
    toBeDeletedKinematicBodies.push_back(id);
}

void MapBasedBodyPointerHolder::queueRigidBodyForDeletion(const ID& id) {
    toBeDeletedRigidBodies.push_back(id);
}

void MapBasedBodyPointerHolder::queueCollidingBodyForDeletion(const ID& id) {
    toBeDeletedCollidingBodies.push_back(id);
}

void MapBasedBodyPointerHolder::queueUserControlledBodyForDeletion(const ID& id) {
    toBeDeletedUserControlledBodies.push_back(id);
}

void MapBasedBodyPointerHolder::registerBodySafely(const ID& id, Body* body) {
    bodiesMap.insert({id, body});
    allBodies.push_back(body);
}

void MapBasedBodyPointerHolder::pruneBodiesList() const {
    for(const auto& toBeDeletedBodyID : toBeDeletedBodies) {
        allBodies.remove_if([toBeDeletedBodyID](Body* body) {
            return body->getBodyID() == toBeDeletedBodyID;
        });
    }
    toBeDeletedBodies.clear();
}

void MapBasedBodyPointerHolder::pruneKinematicBodiesList() const {
    for(const auto& toBeDeletedBodyID : toBeDeletedKinematicBodies) {
        allKinematicBodies.remove_if([toBeDeletedBodyID](KinematicBody* kinematicBody) {
            return kinematicBody->getKinematicBodyID() == toBeDeletedBodyID;
        });
    }
    toBeDeletedKinematicBodies.clear();
}

void MapBasedBodyPointerHolder::pruneRigidBodiesList() const {
    for(const auto& toBeDeletedBodyID : toBeDeletedRigidBodies) {
        allRigidBodies.remove_if([toBeDeletedBodyID](RigidBody* rigidBody) {
            return rigidBody->getRigidBodyID() == toBeDeletedBodyID;
        });
    }
    toBeDeletedRigidBodies.clear();
}

void MapBasedBodyPointerHolder::pruneCollidingBodiesList() const {
    for(const auto& toBeDeletedBodyID : toBeDeletedCollidingBodies) {
        allCollidingBodies.remove_if([toBeDeletedBodyID](CollidingBody* collidingBody) {
            return collidingBody->getCollidingBodyID() == toBeDeletedBodyID;
        });
    }
    toBeDeletedCollidingBodies.clear();
}

void MapBasedBodyPointerHolder::pruneUserControlledBodiesList() const {
    for(const auto& toBeDeletedBodyID : toBeDeletedUserControlledBodies) {
        allUserControlledBodies.remove_if([toBeDeletedBodyID](UserControlledBody* userControlledBody) {
            return userControlledBody->getUserControlledBodyID() == toBeDeletedBodyID;
        });
    }
    toBeDeletedUserControlledBodies.clear();
}

void MapBasedBodyPointerHolder::registerKinematicBodySafely(const ID& id, KinematicBody* kinematicBody) {
    kinematicBodiesMap.insert({id, kinematicBody});
    allKinematicBodies.push_back(kinematicBody);
}

void MapBasedBodyPointerHolder::registerRigidBodySafely(const ID& id, RigidBody* rigidBody) {
    rigidBodiesMap.insert({id, rigidBody});
    allRigidBodies.push_back(rigidBody);
}

void MapBasedBodyPointerHolder::registerCollidingBodySafely(const ID& id, CollidingBody* collidingBody) {
    collidingBodiesMap.insert({id, collidingBody});
    allCollidingBodies.push_back(collidingBody);
}

void MapBasedBodyPointerHolder::registerUserControlledBodySafely(const ID& id, UserControlledBody* collidingBody) {
    userControlledBodiesMap.insert({id, collidingBody});
    allUserControlledBodies.push_back(collidingBody);
}

void MapBasedBodyPointerHolder::removeBodySafely(const ID& id) {
    bodiesMap.erase(id);
    queueBodyForDeletion(id);
}

void MapBasedBodyPointerHolder::removeKinematicBodySafely(const ID& id) {
    kinematicBodiesMap.erase(id);
    queueKinematicBodyForDeletion(id);
}

void MapBasedBodyPointerHolder::removeRigidBodySafely(const ID& id) {
    rigidBodiesMap.erase(id);
    queueRigidBodyForDeletion(id);
}

void MapBasedBodyPointerHolder::removeCollidingBodySafely(const ID& id) {
    collidingBodiesMap.erase(id);
    queueCollidingBodyForDeletion(id);
}

void MapBasedBodyPointerHolder::removeUserControlledBodySafely(const ID& id) {
    userControlledBodiesMap.erase(id);
    queueUserControlledBodyForDeletion(id);
}

Body* MapBasedBodyPointerHolder::getBody(const ID& id) const {
    return bodiesMap.at(id);
}

KinematicBody* MapBasedBodyPointerHolder::getKinematicBody(const ID& id) const {
    return kinematicBodiesMap.at(id);
}

RigidBody* MapBasedBodyPointerHolder::getRigidBody(const ID& id) const {
    return rigidBodiesMap.at(id);
}

CollidingBody* MapBasedBodyPointerHolder::getCollidingBody(const ID& id) const {
    return collidingBodiesMap.at(id);
}

UserControlledBody* MapBasedBodyPointerHolder::getUserControlledBody(const ID& id) const {
    return userControlledBodiesMap.at(id);
}

const std::list<Body*>& MapBasedBodyPointerHolder::getAllBodies() const {
    pruneBodiesList();
    return allBodies;
}

const std::list<KinematicBody*>& MapBasedBodyPointerHolder::getAllKinematicBodies() const {
    pruneKinematicBodiesList();
    return allKinematicBodies;
}

const std::list<RigidBody*>& MapBasedBodyPointerHolder::getAllRigidBodies() const {
    pruneRigidBodiesList();
    return allRigidBodies;
}

const std::list<CollidingBody*>& MapBasedBodyPointerHolder::getAllCollidingBodies() const {
    pruneCollidingBodiesList();
    return allCollidingBodies;
}

const std::list<UserControlledBody*>& MapBasedBodyPointerHolder::getAllUserControlledBodies() const {
    pruneUserControlledBodiesList();
    return allUserControlledBodies;
}
