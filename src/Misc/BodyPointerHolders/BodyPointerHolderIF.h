#pragma once

class Body;
class KinematicBody;
class RigidBody;
class CollidingBody;
class UserControlledBody;
#include "ID.h"
#include <list>
#include <mutex>

class BodyPointerHolderIF {
    public:
        BodyPointerHolderIF() = default;
        virtual ~BodyPointerHolderIF() = default;

        void preventModification();
        void allowModification();

        void registerBody(const ID& id, Body* body);
        void registerKinematicBody(const ID& id, KinematicBody* kinematicBody);
        void registerRigidBody(const ID& id, RigidBody* rigidBody);
        void registerCollidingBody(const ID& id, CollidingBody* collidingBody);
        void registerUserControlledBody(const ID& id, UserControlledBody* userControlledBody);
        void removeBody(const ID& id);
        void removeKinematicBody(const ID& id);
        void removeRigidBody(const ID& id);
        void removeCollidingBody(const ID& id);
        void removeUserControlledBody(const ID& id);

        [[nodiscard]] virtual Body* getBody(const ID& id) const = 0;
        [[nodiscard]] virtual KinematicBody* getKinematicBody(const ID& id) const = 0;
        [[nodiscard]] virtual RigidBody* getRigidBody(const ID& id) const = 0;
        [[nodiscard]] virtual CollidingBody* getCollidingBody(const ID& id) const = 0;
        [[nodiscard]] virtual UserControlledBody* getUserControlledBody(const ID& id) const = 0;

        [[nodiscard]] virtual const std::list<Body*>& getAllBodies() const = 0;
        [[nodiscard]] virtual const std::list<KinematicBody*>& getAllKinematicBodies() const = 0;
        [[nodiscard]] virtual const std::list<RigidBody*>& getAllRigidBodies() const = 0;
        [[nodiscard]] virtual const std::list<CollidingBody*>& getAllCollidingBodies() const = 0;
        [[nodiscard]] virtual const std::list<UserControlledBody*>& getAllUserControlledBodies() const = 0;

    private:
        void lockBodiesModification();
        void unlockBodiesModification();
        void lockKinematicBodiesModification();
        void unlockKinematicBodiesModification();
        void lockRigidBodiesModification();
        void unlockRigidBodiesModification();
        void lockCollidingBodiesModification();
        void unlockCollidingBodiesModification();
        void lockUserControlledBodiesModification();
        void unlockUserControlledBodiesModification();

        virtual void registerBodySafely(const ID& id, Body* body) = 0;
        virtual void registerKinematicBodySafely(const ID& id, KinematicBody* kinematicBody) = 0;
        virtual void registerRigidBodySafely(const ID& id, RigidBody* rigidBody) = 0;
        virtual void registerCollidingBodySafely(const ID& id, CollidingBody* collidingBody) = 0;
        virtual void registerUserControlledBodySafely(const ID& id, UserControlledBody* userControlledBody) = 0;
        virtual void removeBodySafely(const ID& id) = 0;
        virtual void removeKinematicBodySafely(const ID& id) = 0;
        virtual void removeRigidBodySafely(const ID& id) = 0;
        virtual void removeCollidingBodySafely(const ID& id) = 0;
        virtual void removeUserControlledBodySafely(const ID& id) = 0;

        mutable std::mutex bodiesMutex;
        mutable std::mutex kinematicBodiesMutex;
        mutable std::mutex rigidBodiesMutex;
        mutable std::mutex collidingBodiesMutex;
        mutable std::mutex userControlledBodiesMutex;
};
