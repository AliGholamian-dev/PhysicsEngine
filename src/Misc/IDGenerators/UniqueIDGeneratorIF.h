#pragma once

#include "ID.h"

class UniqueIDGeneratorIF {
    public:
        UniqueIDGeneratorIF() = default;
        virtual ~UniqueIDGeneratorIF() = default;

        virtual ID getNewUniqueID() = 0;
};
