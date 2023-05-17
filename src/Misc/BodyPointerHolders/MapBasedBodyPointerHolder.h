#pragma once


#include "BodyPointerHolderIF.h"
#include <map>
#include <vector>

class MapBasedBodyPointerHolder : public BodyPointerHolderIF {
    public:
        MapBasedBodyPointerHolder();
        ~MapBasedBodyPointerHolder() override = default;
        [[nodiscard]] Body* getBody(const ID& id) const override;
        [[nodiscard]] KinematicBody* getKinematicBody(const ID& id) const override;
        [[nodiscard]] RigidBody* getRigidBody(const ID& id) const override;
        [[nodiscard]] CollidingBody* getCollidingBody(const ID& id) const override;
        [[nodiscard]] UserControlledBody* getUserControlledBody(const ID& id) const override;

        [[nodiscard]] const std::list<Body*>& getAllBodies() const override;
        [[nodiscard]] const std::list<KinematicBody*>& getAllKinematicBodies() const override;
        [[nodiscard]] const std::list<RigidBody*>& getAllRigidBodies() const override;
        [[nodiscard]] const std::list<CollidingBody*>& getAllCollidingBodies() const override;
        [[nodiscard]] const std::list<UserControlledBody*>& getAllUserControlledBodies() const override;

		using BodiesMap = std::map<ID, Body*>;
		using KinematicBodiesMap = std::map<ID, KinematicBody*>;
		using RigidBodiesMap = std::map<ID, RigidBody*>;
		using CollidingBodiesMap = std::map<ID, CollidingBody*>;
		using UserControlledBodiesMap = std::map<ID, UserControlledBody*>;

        BodiesMap bodiesMap;
        KinematicBodiesMap kinematicBodiesMap;
        RigidBodiesMap rigidBodiesMap;
        CollidingBodiesMap collidingBodiesMap;
        UserControlledBodiesMap userControlledBodiesMap;

    private:
        void queueBodyForDeletion(const ID& id);
        void queueKinematicBodyForDeletion(const ID& id);
        void queueRigidBodyForDeletion(const ID& id);
        void queueCollidingBodyForDeletion(const ID& id);
        void queueUserControlledBodyForDeletion(const ID& id);

        void pruneBodiesList() const;
        void pruneKinematicBodiesList() const;
        void pruneRigidBodiesList() const;
        void pruneCollidingBodiesList() const;
        void pruneUserControlledBodiesList() const;

        void registerBodySafely(const ID& id, Body* body) override;
        void registerKinematicBodySafely(const ID& id, KinematicBody* kinematicBody) override;
        void registerRigidBodySafely(const ID& id, RigidBody* rigidBody) override;
        void registerCollidingBodySafely(const ID& id, CollidingBody* collidingBody) override;
        void registerUserControlledBodySafely(const ID& id, UserControlledBody* userControlledBody) override;

        void removeBodySafely(const ID& id) override;
        void removeKinematicBodySafely(const ID& id) override;
        void removeRigidBodySafely(const ID& id) override;
        void removeCollidingBodySafely(const ID& id) override;
        void removeUserControlledBodySafely(const ID& id) override;

        mutable std::vector<ID> toBeDeletedBodies;
        mutable std::vector<ID> toBeDeletedKinematicBodies;
        mutable std::vector<ID> toBeDeletedRigidBodies;
        mutable std::vector<ID> toBeDeletedCollidingBodies;
        mutable std::vector<ID> toBeDeletedUserControlledBodies;

        mutable std::list<Body*> allBodies;
        mutable std::list<KinematicBody*> allKinematicBodies;
        mutable std::list<RigidBody*> allRigidBodies;
        mutable std::list<CollidingBody*> allCollidingBodies;
        mutable std::list<UserControlledBody*> allUserControlledBodies;

};
