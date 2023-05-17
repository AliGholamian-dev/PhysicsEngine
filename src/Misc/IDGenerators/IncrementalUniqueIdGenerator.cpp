#include "IncrementalUniqueIdGenerator.h"

ID IncrementalUniqueIDGenerator::idCounter { 0 };

IncrementalUniqueIDGenerator::IncrementalUniqueIDGenerator() : UniqueIDGeneratorIF() {}

ID IncrementalUniqueIDGenerator::getNewUniqueID() {
    incrementIDCounter();
    return idCounter;
}

void IncrementalUniqueIDGenerator::incrementIDCounter() {
    idCounter++;
}
