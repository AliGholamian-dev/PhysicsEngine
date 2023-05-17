#pragma once


#include "PointerHelper.h"
#include "ID.h"
#include "LayerMask.h"
#include "CollisionResolutionInfo.h"
#include "CollisionResponseInfo.h"
#include <vector>
class Body;
class KinematicBody;
class RigidBody;
class BodyPointerHolderIF;

class CollidingBody {
    public:
        CollidingBody() = delete;
        CollidingBody(const CollidingBody& other) = delete;
        CollidingBody(CollidingBody&& other) noexcept  = delete;
        CollidingBody(ID id,
                      const LayerMask::Value& collisionLayer,
                      Body* body,
                      KinematicBody* kinematicBody,
                      RigidBody* rigidBody,
                      const wp<BodyPointerHolderIF>& bodyPointerHolder);
        virtual ~CollidingBody();

        void setLayerMask(const LayerMask& newLayerMask);
        virtual void doUserTasksOnCollisionDetection(const CollidingBody* collidingWithBody,
                                                     const CollisionResolutionInfo& collisionResolutionInfo);

        virtual void doUserTasksAfterCollisionResolution(const CollidingBody* resolvedWithBody,
                                                         const CollisionResponseInfo& collisionResponseInfo);
        virtual void doUserTasksAfterCollisionResponse(const CollidingBody* respondedToBody,
                                                       const CollisionResponseInfo& collisionResponseInfo);

        [[nodiscard]] const ID& getCollidingBodyID() const;
        [[nodiscard]] const LayerMask::Value& getCollisionLayer() const;
        [[nodiscard]] const LayerMask& getLayerMask() const;
        Body* getBody();
        KinematicBody* getKinematicBody();
        RigidBody* getRigidBody();

    private:
        void registerBody();
        void removeBody();

        const ID id;
        const LayerMask::Value collisionLayer;
        Body* body;
        KinematicBody* kinematicBody;
        RigidBody* rigidBody;
        wp<BodyPointerHolderIF> bodyPointerHolder;

        LayerMask layerMask;
};
