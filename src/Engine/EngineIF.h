#pragma once


#include "PointerHelper.h"
#include "CollisionResolutionInfo.h"
#include "CollisionResponseInfo.h"
#include <vector>
#include <list>
class CollisionDetectorIF;
class CollisionResolverIF;
class CollisionResponseIF;
class BodyPointerHolderIF;
class UserControlledBody;
class Constraint;

class EngineIF {
    public:
        enum class RunMode{
            runEachStageSeparatelyOnAllObjects = 0,
            runAllStagesOnEachObject = 1,
            updateAllThenRunCollisionStagesOnEachObject = 2
        };
        EngineIF() = delete;
        EngineIF(RunMode runMode,
                 int numberOfSubSteps,
                 int numberOfConstraintIterations,
                 const sp<CollisionDetectorIF>& collisionDetectorIF,
                 const sp<CollisionResolverIF>& collisionResolverIF,
                 const sp<CollisionResponseIF>& collisionResponseIF,
                 const wp<BodyPointerHolderIF>& bodyPointerHolderIF);
        virtual ~EngineIF() = default;
        void updateConstraints(const double& deltaTime);
        virtual void runEngine(const double& deltaTime);

        void addConstraint(const wp<Constraint>& constraint);

    private:
        void checkInputParameters() const;

        virtual void prepareUserControlledBodies();
        virtual bool checkForNextUserControlledBodyAvailable();
        virtual UserControlledBody* provideNextUserControlledBod();
        static void manipulateEachUserControlledBodies(UserControlledBody* userControlledBody, const double& deltaTime);
        void manipulateAllUserControlledBodies(const double& deltaTime);

        virtual void prepareBodiesForUpdate();
        virtual bool checkForNextBodyAvailableForUpdate();
        virtual void updateNextBody(const double& deltaTime);
        void updateAllBodies(const double& deltaTime);

        virtual void doTasksBeforeUpdateAndCollisionStage(const double& deltaTime);
        virtual void doTasksAfterUpdateAndCollisionStage(const double& deltaTime);

        static void doUserTasksOnCollisionDetection(const CollisionResolutionInfo& collisionResolutionInfo,
                                                    const double& deltaTime);
        static void doUserTasksOnCollisionResolution(const CollisionResponseInfo& collisionResponseInfo,
                                                     const double& deltaTime);
        static void doUserTasksOnCollisionResponse(const CollisionResponseInfo& collisionResponseInfo,
                                                   const double& deltaTime);

        void runEachStageSeparatelyOnAllObjects(const double& deltaTime);
        void runAllStagesOnEachObject(const double& deltaTime);
        void updateAllThenRunCollisionStagesOnEachObject(const double& deltaTime);

        const RunMode runMode;
        const int numberOfSubSteps;
        const int numberOfConstraintIterations;
        sp<CollisionDetectorIF> collisionDetectorIF;
        sp<CollisionResolverIF> collisionResolverIF;
        sp<CollisionResponseIF> collisionResponseIF;
        wp<BodyPointerHolderIF> bodyPointerHolderIF;
        sp<BodyPointerHolderIF> bodyPointerHolder;
        std::list<wp<Constraint>> constraints;
};
