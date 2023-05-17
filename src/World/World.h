#pragma once


#include <chrono>
#include "PointerHelper.h"
class EngineIF;

class World {
    public:
        World() = default;
        virtual ~World()  = default;
        void simulate();

    protected:
        void setEngine(sp<EngineIF> worldEngine);

    private:
        virtual void initialize();
        virtual void simulateUserWorld();
		void updateLastRunTime(const std::chrono::system_clock::time_point& now);
        double calculateElapsedTime();

        sp<EngineIF> engine;
        bool initializedOnce { false };
		optional<std::chrono::system_clock::time_point> lastRunTime;
};
