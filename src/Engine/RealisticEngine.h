#pragma once


#include "EngineIF.h"
#include "PointerHelper.h"
#include "Vector2D.h"
#include "MapBasedBodyPointerHolder.h"
#include <vector>
class RigidBody;

class RealisticEngine final : public EngineIF {
    public:
        RealisticEngine() = delete;
        RealisticEngine(RunMode runMode,
                        int numberOfIterations,
                        int numberOfConstraintIterations,
                        const Vector2D& gravity,
                        const sp<CollisionDetectorIF>& collisionDetectorIF,
                        const sp<CollisionResolverIF>& collisionResolverIF,
                        const sp<CollisionResponseIF>& collisionResponseIF,
                        const wp<MapBasedBodyPointerHolder>& mapBasedBodyPointerHolder);
        ~RealisticEngine() override = default;

    private:
        void applyGravity(RigidBody* rigidBody);
        static void manipulateRigidBody(RigidBody* rigidBody, const double& deltaTime);

        void prepareUserControlledBodies() override;
        bool checkForNextUserControlledBodyAvailable() override;
        UserControlledBody* provideNextUserControlledBod() override;
        void prepareBodiesForUpdate() override;
        bool checkForNextBodyAvailableForUpdate() override;
        void updateNextBody(const double& deltaTime) override;

        const Vector2D gravity;
        wp<MapBasedBodyPointerHolder> mapBasedBodyPointerHolder;
        sp<MapBasedBodyPointerHolder> bodyHolderForUserControlledBodies;
        sp<MapBasedBodyPointerHolder> bodyHolderForRigidBodies;
        MapBasedBodyPointerHolder::UserControlledBodiesMap::iterator userControlledIt;
        MapBasedBodyPointerHolder::RigidBodiesMap::iterator rigidIt;
};
