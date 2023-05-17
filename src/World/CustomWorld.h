#pragma once


#include <vector>
#include "PositionConstraint.h"
#include "RealisticEngine.h"
#include "World.h"
#include "Singleton.h"
#include "Position2D.h"
#include "MapBasedBodyPointerHolder.h"
#include "NewtonAccelerationCalculator.h"
#include "ZeroInitialStateKinematicEquation.h"
#include "UserControlledBody.h"
#include "CollidingBody.h"
#include "RigidBody.h"
#include "KinematicBody.h"
#include "Body.h"
#include "IncrementalUniqueIdGenerator.h"

///TODO: This file in dirty and just for test: ignore for now

class MovingObject : public Body, public KinematicBody, public RigidBody, public CollidingBody
{
    public:
        MovingObject(ID id,
                     BodyType bodyType,
                     const Position2D& initialPosition,
                     const std::vector<Position2D>& clockwiseVerticesRelativeToCenter,
                     double radius,
                     const std::vector<TriangleInfo>& triangles,
                     RigidInfo rigidInfo,
                     const wp<MapBasedBodyPointerHolder>& mapBasedBodyPointerHolder) :
                Body(id,
                     bodyType,
                     initialPosition,
                     0.0,
                     clockwiseVerticesRelativeToCenter,
                     radius,
                     triangles,
                     mapBasedBodyPointerHolder),
                KinematicBody(id,
                              this,
                              std::make_shared<ZeroInitialStateKinematicEquation>(),
                              {{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, 0.0, 0.0 , 0.0},
                              {0.9, 0.9},
                              mapBasedBodyPointerHolder),
                RigidBody(id,
                          RigidBodyType::notStaticBody,
                          this,
                          std::make_shared<NewtonAccelerationCalculator>(this, this),
                          rigidInfo,
                          10.0,
                          mapBasedBodyPointerHolder),
                CollidingBody(id,
                              LayerMask::Value::layer1,
                              this,
                              this,
                              this,
                              mapBasedBodyPointerHolder)
        {}
        ~MovingObject() override = default;

        void doUserTasksOnCollisionDetection(const CollidingBody* collidingWithBody,
                                             const CollisionResolutionInfo& collisionResolutionInfo) override
        {
            collide = true;
        }

        bool getCollisionStatus() {
            bool collisionStatus = collide;
            collide = false;
            return collisionStatus;
        }

    private:
        bool collide { false };
};

class StaticObject : public Body, public RigidBody, public CollidingBody
{
    public:
        StaticObject(ID id,
                     BodyType bodyType,
                     const Position2D& initialPosition,
                     const std::vector<Position2D>& clockwiseVerticesRelativeToCenter,
                     double radius,
                     const std::vector<TriangleInfo>& triangles,
                     RigidInfo rigidInfo,
                     const wp<MapBasedBodyPointerHolder>& mapBasedBodyPointerHolder) :
                Body(id,
                     bodyType,
                     initialPosition,
                     0.0,
                     clockwiseVerticesRelativeToCenter,
                     radius,
                     triangles,
                     mapBasedBodyPointerHolder),
                RigidBody(id,
                          RigidBodyType::staticBody,
                          nullptr,
                          std::make_shared<NewtonAccelerationCalculator>(this, this),
                          rigidInfo,
                          0.0,
                          mapBasedBodyPointerHolder),
                CollidingBody(id,
                              LayerMask::Value::layer1,
                              this,
                              nullptr,
                              this,
                              mapBasedBodyPointerHolder)
        {}
        ~StaticObject() override = default;
        void doUserTasksOnCollisionDetection(const CollidingBody* collidingWithBody,
                                             const CollisionResolutionInfo& collisionResolutionInfo) override
        {
            collide = true;
        }

        bool getCollisionStatus() {
            bool collisionStatus = collide;
            collide = false;
            return collisionStatus;
        }

    private:
        bool collide { false };
};

class UserObject : public Body, public KinematicBody, public RigidBody, public CollidingBody, public UserControlledBody
{
    public:
        UserObject(ID id,
                   BodyType bodyType,
                   const Position2D& initialPosition,
                   const std::vector<Position2D>& clockwiseVerticesRelativeToCenter,
                   double radius,
                   const std::vector<TriangleInfo>& triangles,
                   RigidInfo rigidInfo,
                   const wp<MapBasedBodyPointerHolder>& mapBasedBodyPointerHolder) :
                Body(id,
                     bodyType,
                     initialPosition,
                     0,
                     clockwiseVerticesRelativeToCenter,
                     radius,
                     triangles,
                     mapBasedBodyPointerHolder),
                KinematicBody(id,
                              this,
                              std::make_shared<ZeroInitialStateKinematicEquation>(),
                              {{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, 0.0, 0.0 , 0.0},
                              {0.9, 0.9},
                              mapBasedBodyPointerHolder),
                RigidBody(id,
                          RigidBodyType::notStaticBody,
                          this,
                          std::make_shared<NewtonAccelerationCalculator>(this, this),
                          rigidInfo,
                          0.0,
                          mapBasedBodyPointerHolder),
                CollidingBody(id,
                              LayerMask::Value::layer2,
                              this,
                              this,
                              this,
                              mapBasedBodyPointerHolder),
                UserControlledBody(id, mapBasedBodyPointerHolder)
        {}
        ~UserObject() override = default;
        void doUserTasks(const double& deltaTime) override {
            this->addForce(moveVector);
            this->rotate( angle);
            angle = 0;
        }

        void doUserTasksOnCollisionDetection(const CollidingBody* collidingWithBody,
                                             const CollisionResolutionInfo& collisionResolutionInfo) override
        {
            collide = true;
        }

        bool getCollisionStatus() {
            bool collisionStatus = collide;
            collide = false;
            return collisionStatus;
        }
        void setMoveVector(Vector2D newMoveVector) {
            moveVector = std::move(newMoveVector);
        }
        void rot(double ang) {
            angle = ang;
        }

    private:
        bool collide { false };
        Vector2D moveVector;
        double angle { 0 };
};

class CustomWorld : public World, public Singleton<CustomWorld> {
    private:
        friend class Singleton<CustomWorld>;
        CustomWorld();
        void initialize() override;
        void simulateUserWorld() override;
        sp<MapBasedBodyPointerHolder> mapBasedBodyPointerHolder;
        up<IncrementalUniqueIDGenerator> idGenerator;

    public:
        ~CustomWorld() override = default;
        void createMovingObject(const Position2D& position);
        void createStaticObject(const Position2D& position);
        std::vector<MovingObject*> movingObjects;
        std::vector<StaticObject*> staticObjects;
        std::vector<sp<PositionConstraint>> positionConstraints;
        sp<RealisticEngine> realisticEngine;
        UserObject* userObject = nullptr;
};
