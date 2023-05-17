#include "BodyPointerHolderIF.h"

void BodyPointerHolderIF::lockBodiesModification() {
    bodiesMutex.lock();
}

void BodyPointerHolderIF::unlockBodiesModification() {
    bodiesMutex.unlock();
}

void BodyPointerHolderIF::lockKinematicBodiesModification() {
    kinematicBodiesMutex.lock();
}

void BodyPointerHolderIF::unlockKinematicBodiesModification() {
    kinematicBodiesMutex.unlock();
}

void BodyPointerHolderIF::lockRigidBodiesModification() {
    rigidBodiesMutex.lock();
}

void BodyPointerHolderIF::unlockRigidBodiesModification() {
    rigidBodiesMutex.unlock();
}

void BodyPointerHolderIF::lockCollidingBodiesModification() {
    collidingBodiesMutex.lock();
}

void BodyPointerHolderIF::unlockCollidingBodiesModification() {
    collidingBodiesMutex.unlock();
}

void BodyPointerHolderIF::lockUserControlledBodiesModification() {
    userControlledBodiesMutex.lock();
}

void BodyPointerHolderIF::unlockUserControlledBodiesModification() {
    userControlledBodiesMutex.unlock();
}


void BodyPointerHolderIF::preventModification() {
    lockBodiesModification();
    lockKinematicBodiesModification();
    lockRigidBodiesModification();
    lockCollidingBodiesModification();
    lockUserControlledBodiesModification();
}

void BodyPointerHolderIF::allowModification() {
    unlockBodiesModification();
    unlockKinematicBodiesModification();
    unlockRigidBodiesModification();
    unlockCollidingBodiesModification();
    unlockUserControlledBodiesModification();
}

void BodyPointerHolderIF::registerBody(const ID& id, Body* body) {
    std::lock_guard<std::mutex> guard(bodiesMutex);
    registerBodySafely(id, body);
}

void BodyPointerHolderIF::registerKinematicBody(const ID& id, KinematicBody* kinematicBody) {
    std::lock_guard<std::mutex> guard(kinematicBodiesMutex);
    registerKinematicBodySafely(id, kinematicBody);
}

void BodyPointerHolderIF::registerRigidBody(const ID& id, RigidBody* rigidBody) {
    std::lock_guard<std::mutex> guard(rigidBodiesMutex);
    registerRigidBodySafely(id, rigidBody);
}

void BodyPointerHolderIF::registerCollidingBody(const ID& id, CollidingBody* collidingBody) {
    std::lock_guard<std::mutex> guard(collidingBodiesMutex);
    registerCollidingBodySafely(id, collidingBody);
}

void BodyPointerHolderIF::registerUserControlledBody(const ID& id, UserControlledBody* userControlledBody) {
    std::lock_guard<std::mutex> guard(userControlledBodiesMutex);
    registerUserControlledBodySafely( id, userControlledBody);
}

void BodyPointerHolderIF::removeBody(const ID& id) {
    std::lock_guard<std::mutex> guard(bodiesMutex);
    removeBodySafely(id);
}

void BodyPointerHolderIF::removeKinematicBody(const ID& id) {
    std::lock_guard<std::mutex> guard(kinematicBodiesMutex);
    removeKinematicBodySafely(id);
}

void BodyPointerHolderIF::removeRigidBody(const ID& id) {
    std::lock_guard<std::mutex> guard(rigidBodiesMutex);
    removeRigidBodySafely(id);
}

void BodyPointerHolderIF::removeCollidingBody(const ID& id) {
    std::lock_guard<std::mutex> guard(collidingBodiesMutex);
    removeCollidingBodySafely(id);
}

void BodyPointerHolderIF::removeUserControlledBody(const ID& id) {
    std::lock_guard<std::mutex> guard(userControlledBodiesMutex);
    removeUserControlledBodySafely(id);
}
