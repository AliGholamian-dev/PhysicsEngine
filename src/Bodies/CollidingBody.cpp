#include "CollidingBody.h"
#include "BodyPointerHolderIF.h"


CollidingBody::CollidingBody(ID id,
                             const LayerMask::Value& collisionLayer,
                             Body* body,
                             KinematicBody* kinematicBody,
                             RigidBody* rigidBody,
                             const wp<BodyPointerHolderIF>& bodyPointerHolder) :
        id(id),
        collisionLayer(collisionLayer),
        body(body),
        kinematicBody(kinematicBody),
        rigidBody(rigidBody),
        bodyPointerHolder(bodyPointerHolder),
        layerMask(0)
{
    registerBody();
}

CollidingBody::~CollidingBody() {
    removeBody();
}

void CollidingBody::registerBody() {
    if(!bodyPointerHolder.expired()) {
        bodyPointerHolder.lock()->registerCollidingBody(id, this);
    }
}

void CollidingBody::removeBody() {
    if(!bodyPointerHolder.expired()) {
        bodyPointerHolder.lock()->removeCollidingBody(id);
    }
}

void CollidingBody::setLayerMask(const LayerMask& newLayerMask) {
    layerMask = newLayerMask;
}

void CollidingBody::doUserTasksOnCollisionDetection(const CollidingBody* collidingWithBody,
                                                    const CollisionResolutionInfo& collisionResolutionInfo) {}

void CollidingBody::doUserTasksAfterCollisionResolution(const CollidingBody* resolvedWithBody,
                                                        const CollisionResponseInfo& collisionResponseInfo)  {}

void CollidingBody::doUserTasksAfterCollisionResponse(const CollidingBody* respondedToBody,
                                                      const CollisionResponseInfo& collisionResponseInfo) {}

const ID& CollidingBody::getCollidingBodyID() const {
    return id;
}

const LayerMask::Value& CollidingBody::getCollisionLayer() const {
    return collisionLayer;
}

const LayerMask& CollidingBody::getLayerMask() const {
    return layerMask;
}

Body* CollidingBody::getBody() {
    return body;
}

KinematicBody* CollidingBody::getKinematicBody() {
    return kinematicBody;
}

RigidBody* CollidingBody::getRigidBody() {
    return rigidBody;
}

