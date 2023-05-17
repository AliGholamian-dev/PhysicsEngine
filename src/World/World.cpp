#include "World.h"
#include "EngineIF.h"


void World::setEngine(sp<EngineIF> worldEngine) {
    engine = worldEngine;
}

void World::initialize() {}

void World::simulateUserWorld() {}

void World::updateLastRunTime(const std::chrono::system_clock::time_point& now) {
    lastRunTime = now;
}

double World::calculateElapsedTime() {
    double elapsedTimeInSec { 0.0 };
    const auto now = std::chrono::high_resolution_clock::now();
    if(lastRunTime) {
        std::chrono::duration<double> elapsedTime =  now - *lastRunTime;
        elapsedTimeInSec = elapsedTime.count();
    }
    updateLastRunTime(now);
    return elapsedTimeInSec;
}

void World::simulate() {
    double deltaTime = calculateElapsedTime();
    if(!initializedOnce) {
        initialize();
        initializedOnce = true;
    }
    simulateUserWorld();
    if(engine) {
        engine->runEngine(deltaTime);
    }
}
