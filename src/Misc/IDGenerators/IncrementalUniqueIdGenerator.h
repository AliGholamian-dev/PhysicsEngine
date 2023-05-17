#pragma once


#include "UniqueIDGeneratorIF.h"

class IncrementalUniqueIDGenerator : public UniqueIDGeneratorIF {
    public:
        IncrementalUniqueIDGenerator();
        ~IncrementalUniqueIDGenerator() override = default;

        ID getNewUniqueID() override;

    private:
        static void incrementIDCounter();
        static ID idCounter;

};
