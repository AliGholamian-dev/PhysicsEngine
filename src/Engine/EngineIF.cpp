#include "EngineIF.h"
#include "Exceptions.h"
#include "Constraint.h"
#include "BodyPointerHolderIF.h"
#include "CollisionDetectorIF.h"
#include "CollisionResolverIF.h"
#include "CollisionResponseIF.h"
#include "UserControlledBody.h"
#include "CollidingBody.h"


EngineIF::EngineIF(RunMode runMode,
                   int numberOfSubSteps,
                   int numberOfConstraintIterations,
                   const sp<CollisionDetectorIF>& collisionDetectorIF,
                   const sp<CollisionResolverIF>& collisionResolverIF,
                   const sp<CollisionResponseIF>& collisionResponseIF,
                   const wp<BodyPointerHolderIF>& bodyPointerHolderIF) :
        runMode(runMode),
        numberOfSubSteps(numberOfSubSteps),
        numberOfConstraintIterations(numberOfConstraintIterations),
        collisionDetectorIF(collisionDetectorIF),
        collisionResolverIF(collisionResolverIF),
        collisionResponseIF(collisionResponseIF),
        bodyPointerHolderIF(bodyPointerHolderIF)

{
    checkInputParameters();
}

void EngineIF::checkInputParameters() const {
    if(numberOfSubSteps <= 0) {
        throw WrongParametersException();
    }
}

void EngineIF::prepareUserControlledBodies()                {               }
bool EngineIF::checkForNextUserControlledBodyAvailable()    { return false; }
UserControlledBody*EngineIF::provideNextUserControlledBod() { return nullptr; }

void EngineIF::manipulateEachUserControlledBodies(UserControlledBody* userControlledBody,
                                                  const double& deltaTime)
{
    if(userControlledBody) {
        userControlledBody->doUserTasks(deltaTime);
    }
}

void EngineIF::manipulateAllUserControlledBodies(const double& deltaTime) {
    prepareUserControlledBodies();
    while(checkForNextUserControlledBodyAvailable()) {
        const auto userControlledBody = provideNextUserControlledBod();
        manipulateEachUserControlledBodies(userControlledBody, deltaTime);
    }
}

void EngineIF::doTasksBeforeUpdateAndCollisionStage(const double& deltaTime) {}
void EngineIF::doTasksAfterUpdateAndCollisionStage(const double& deltaTime)  {}

void EngineIF::prepareBodiesForUpdate()                 {               }
bool EngineIF::checkForNextBodyAvailableForUpdate()     { return false; }
void EngineIF::updateNextBody(const double& deltaTime)  {               }
void EngineIF::updateAllBodies(const double& deltaTime) {
    prepareBodiesForUpdate();
    while (checkForNextBodyAvailableForUpdate()) {
        updateNextBody(deltaTime);
    }
}

void
EngineIF::doUserTasksOnCollisionDetection(const CollisionResolutionInfo& collisionResolutionInfo,
                                          const double& deltaTime)
{
    const auto& firstCollidingBody = collisionResolutionInfo.collidingPair.first;
    const auto& secondCollidingBody = collisionResolutionInfo.collidingPair.second;
    firstCollidingBody->doUserTasksOnCollisionDetection(secondCollidingBody, collisionResolutionInfo);
    secondCollidingBody->doUserTasksOnCollisionDetection(firstCollidingBody, collisionResolutionInfo);
}

void
EngineIF::doUserTasksOnCollisionResolution(const CollisionResponseInfo& collisionResponseInfo,
                                           const double& deltaTime)
{
    const auto& firstCollidingBody = collisionResponseInfo.collidingPair.first;
    const auto& secondCollidingBody = collisionResponseInfo.collidingPair.second;
    firstCollidingBody->doUserTasksAfterCollisionResolution(secondCollidingBody, collisionResponseInfo);
    secondCollidingBody->doUserTasksAfterCollisionResolution(firstCollidingBody, collisionResponseInfo);
}

void
EngineIF::doUserTasksOnCollisionResponse(const CollisionResponseInfo& collisionResponseInfo,
                                         const double& deltaTime)
{
    const auto& firstCollidingBody = collisionResponseInfo.collidingPair.first;
    const auto& secondCollidingBody = collisionResponseInfo.collidingPair.second;
    firstCollidingBody->doUserTasksAfterCollisionResolution(secondCollidingBody, collisionResponseInfo);
    secondCollidingBody->doUserTasksAfterCollisionResolution(firstCollidingBody, collisionResponseInfo);
}

void EngineIF::runEachStageSeparatelyOnAllObjects(const double& deltaTime) {
    updateAllBodies(deltaTime);

    if(collisionDetectorIF) {
        const auto possibleCollidingBodies = collisionDetectorIF->doBroadPhaseOnAllPairs(deltaTime);
        const auto collidingBodies = collisionDetectorIF->doNarrowPhaseOnAllPairs(possibleCollidingBodies, deltaTime);
        for(const auto& collidingBody : collidingBodies) {
            doUserTasksOnCollisionDetection(collidingBody, deltaTime);
        }
        if(collisionResolverIF) {
            const auto collisionResponseInfos = collisionResolverIF->resolveAllCollidingPairs(collidingBodies, deltaTime);
            for(const auto& collisionResponseInfo : collisionResponseInfos) {
                doUserTasksOnCollisionResolution(collisionResponseInfo, deltaTime);
            }
            if(collisionResponseIF) {
                collisionResponseIF->respondToAllCollidingBodies(collisionResponseInfos, deltaTime);
                for(const auto& collisionResponseInfo : collisionResponseInfos) {
                    doUserTasksOnCollisionResponse(collisionResponseInfo, deltaTime);
                }
            }
        }
    }
}

void EngineIF::runAllStagesOnEachObject(const double& deltaTime) {
    prepareBodiesForUpdate();
    while (checkForNextBodyAvailableForUpdate()) {
        updateNextBody(deltaTime);
        if(collisionDetectorIF) {
            const auto possibleCollidingBodies = collisionDetectorIF->doBroadPhaseOnAllPairs(deltaTime);
            for(const auto& collidingPair : possibleCollidingBodies) {
                const auto collisionResolutionInfo = collisionDetectorIF->doNarrowPhaseOnEachPair(collidingPair, deltaTime);
                if(collisionResolutionInfo) {
                    doUserTasksOnCollisionDetection(*collisionResolutionInfo, deltaTime);
                    if(collisionResolverIF) {
                        const auto collisionResponseInfo = collisionResolverIF->resolveEachCollidingPair(*collisionResolutionInfo, deltaTime);
                        if(collisionResponseInfo) {
                            doUserTasksOnCollisionResolution(*collisionResponseInfo, deltaTime);
                            if(collisionResponseIF) {
                                collisionResponseIF->respondToEachCollidingPair(*collisionResponseInfo, deltaTime);
                                doUserTasksOnCollisionResponse(*collisionResponseInfo, deltaTime);
                            }
                        }
                    }
                }
            }
        }
    }
}

void EngineIF::updateAllThenRunCollisionStagesOnEachObject(const double& deltaTime) {
    updateAllBodies(deltaTime);

    if(collisionDetectorIF) {
        const auto possibleCollidingBodies = collisionDetectorIF->doBroadPhaseOnAllPairs(deltaTime);
        for(const auto& collidingPair : possibleCollidingBodies) {
            const auto collisionResolutionInfo = collisionDetectorIF->doNarrowPhaseOnEachPair(collidingPair, deltaTime);
            if(collisionResolutionInfo) {
                doUserTasksOnCollisionDetection(*collisionResolutionInfo, deltaTime);
                if(collisionResolverIF) {
                    const auto collisionResponseInfo = collisionResolverIF->resolveEachCollidingPair(*collisionResolutionInfo, deltaTime);
                    if(collisionResponseInfo) {
                        doUserTasksOnCollisionResolution(*collisionResponseInfo, deltaTime);
                        if(collisionResponseIF) {
                            collisionResponseIF->respondToEachCollidingPair(*collisionResponseInfo, deltaTime);
                            doUserTasksOnCollisionResponse(*collisionResponseInfo, deltaTime);
                        }
                    }
                }
            }
        }
    }
}

void EngineIF::updateConstraints(const double& deltaTime) {
    for(auto it = constraints.begin(); it != constraints.end();) {
        auto& constraintWeakPointer = *it;
        if(constraintWeakPointer.expired()) {
            it = constraints.erase(it);
        }
        else {
            auto constraint = constraintWeakPointer.lock();
            if(constraint) {
                constraint->updateConstraint(deltaTime);
                it++;
            }
            else {
                it = constraints.erase(it);
            }
        }

    }
}

void EngineIF::runEngine(const double& deltaTime) {
    if(!bodyPointerHolderIF.expired()) {
        bodyPointerHolder = bodyPointerHolderIF.lock();
        if(bodyPointerHolder) {
            bodyPointerHolder->preventModification();

            const auto subStepTime = deltaTime / numberOfSubSteps;
            for (int i = 0; i < numberOfSubSteps; ++i) {
                doTasksBeforeUpdateAndCollisionStage(subStepTime);
                manipulateAllUserControlledBodies(subStepTime);

                const auto iterationTime = subStepTime / numberOfConstraintIterations;
                for (int j = 0; j < numberOfConstraintIterations; ++j) {
                    updateConstraints(iterationTime);
                }

                switch (runMode) {
                    case RunMode::runEachStageSeparatelyOnAllObjects: {
                        runEachStageSeparatelyOnAllObjects(subStepTime);
                        break;
                    }
                    case RunMode::runAllStagesOnEachObject: {
                        runAllStagesOnEachObject(subStepTime);
                        break;
                    }
                    case RunMode::updateAllThenRunCollisionStagesOnEachObject: {
                        updateAllThenRunCollisionStagesOnEachObject(subStepTime);
                        break;
                    }
                    default:
                        break;
                }

                doTasksAfterUpdateAndCollisionStage(subStepTime);
            }

            bodyPointerHolder->allowModification();
        }
        bodyPointerHolder.reset();
    }
}

void EngineIF::addConstraint(const wp<Constraint>& constraint) {
    constraints.push_back(constraint);
}
