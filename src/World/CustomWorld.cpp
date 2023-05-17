#include "CustomWorld.h"
#include "SATCollisionDetector.h"
#include "MassBasedCollisionResolver.h"
#include "ImpulseBasedCollisionResponse.h"
#include "PolygonHelper.h"
#include "CircleHelper.h"

///TODO: This file in dirty and just for test: ignore for now

CustomWorld::CustomWorld() :
        World(),
        mapBasedBodyPointerHolder(std::make_shared<MapBasedBodyPointerHolder>()),
        idGenerator(std::make_unique<IncrementalUniqueIDGenerator>()) {}

void CustomWorld::initialize() {
    Vector2D gravity {0, 10};
    realisticEngine = std::make_shared<RealisticEngine>(
            EngineIF::RunMode::updateAllThenRunCollisionStagesOnEachObject,
            10,
            10,
            gravity,
            std::make_shared<SATCollisionDetector>(mapBasedBodyPointerHolder),
            std::make_shared<MassBasedCollisionResolver>(),
            std::make_shared<ImpulseBasedCollisionResponse>(),
            mapBasedBodyPointerHolder);
    setEngine(realisticEngine);
    //    std::vector<Position2D> vertices {{-30, 30}, {30, 30}, {30, -30}, {-30, -30}};
    std::vector<Position2D> vertices {{0, 0}, {-30, -30}, {0, 50}, {30, -30}};
    vertices = PolygonHelper::getRelativeVertices(vertices);
    auto triangles = PolygonHelper::getTriangles(vertices);
    RigidInfo rigidInfo {0, 0, 0.001, 0.5, 0, 0.9, 0.8};
    rigidInfo.area = PolygonHelper::getArea(triangles);
    rigidInfo.mass = PolygonHelper::getMass(rigidInfo.area, rigidInfo.density);
    rigidInfo.rotationalInertia = PolygonHelper::getMomentOfInertiaByRelativeVertices(vertices, rigidInfo.density);
    userObject = new UserObject(idGenerator->getNewUniqueID(),
                                Body::BodyType::polygon,
                                {0, 0},
                                vertices,
                                0,
                                triangles,
                                rigidInfo,
                                mapBasedBodyPointerHolder);
    LayerMask layerMask(LayerMask::Value::layer2 | LayerMask::layer1);
    userObject->setLayerMask(layerMask);
}

void CustomWorld::simulateUserWorld() {}

void CustomWorld::createMovingObject(const Position2D& position)
{
    int i = std::rand() % 2;
    MovingObject* newMovingObject;
    if(i) {
        std::vector<Position2D> vertices {{-10, 10}, {10, 10}, {10, -10}, {-10, -10}};
        auto triangles = PolygonHelper::getTriangles(vertices);
        RigidInfo rigidInfo {0, 0, 0.00001, 0.5, 0, 0.9, 0.8};
        rigidInfo.area = PolygonHelper::getArea(triangles);
        rigidInfo.mass = PolygonHelper::getMass(rigidInfo.area, rigidInfo.density);
        rigidInfo.rotationalInertia = PolygonHelper::getMomentOfInertiaByRelativeVertices(vertices, rigidInfo.density);
        newMovingObject = new MovingObject(idGenerator->getNewUniqueID(),
                                           Body::BodyType::polygon,
                                           position,
                                           vertices,
                                           0,
                                           triangles,
                                           rigidInfo,
                                           mapBasedBodyPointerHolder);
    }
    else {
        RigidInfo rigidInfo {0, 0, 0.00001, 0.5, 0, 0.9, 0.8};
        rigidInfo.area = CircleHelper::getArea(10);
        rigidInfo.mass = CircleHelper::getMassByArea(rigidInfo.area, rigidInfo.density);
        rigidInfo.rotationalInertia = CircleHelper::getMomentOfInertia(rigidInfo.mass, 10);
        newMovingObject = new MovingObject(idGenerator->getNewUniqueID(),
                                           Body::BodyType::circle,
                                           position,
                                           {},
                                           10,
                                           {},
                                           rigidInfo,
                                           mapBasedBodyPointerHolder);
    }
    LayerMask layerMask(LayerMask::Value::layer1 | LayerMask::layer2);
    newMovingObject->setLayerMask(layerMask);
    newMovingObject->setCenterPosition(userObject->getCenterPosition() + Vector2D(200, 0));
    sp<PositionConstraint> positionConstraint = std::make_shared<PositionConstraint>(userObject,
                                                                                     userObject,
                                                                                     userObject,
                                                                                     newMovingObject,
                                                                                     newMovingObject,
                                                                                     newMovingObject,
                                                                                     200);
    positionConstraints.push_back(positionConstraint);
    realisticEngine->addConstraint(positionConstraint);
    movingObjects.push_back(std::move(newMovingObject));
}

void CustomWorld::createStaticObject(const Position2D& position)
{
    int i = std::rand() % 2;
    StaticObject* newStaticObject;
    if(i) {
        std::vector<Position2D> vertices {{-500, 100}, {500, 100}, {500, -100}, {-500, -100}};
        auto triangles = PolygonHelper::getTriangles(vertices);
        RigidInfo rigidInfo {std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), 1, 1, std::numeric_limits<double>::max(), 0.8, 0.7};
        newStaticObject = new StaticObject(idGenerator->getNewUniqueID(),
                                           Body::BodyType::polygon,
                                           position,
                                           vertices,
                                           0,
                                           triangles,
                                           rigidInfo,
                                           mapBasedBodyPointerHolder);
    }
    else {
        RigidInfo rigidInfo {std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), 1, 1, std::numeric_limits<double>::max(), 0.8, 0.7};
        newStaticObject = new StaticObject(idGenerator->getNewUniqueID(),
                                           Body::BodyType::circle,
                                           position,
                                           {},
                                           100,
                                           {},
                                           rigidInfo,
                                           mapBasedBodyPointerHolder);
    }
    LayerMask layerMask(LayerMask::Value::layer1 | LayerMask::layer2);
    newStaticObject->setLayerMask(layerMask);
    newStaticObject->rotate(3.1415 / 20);
    staticObjects.push_back(std::move(newStaticObject));
}
